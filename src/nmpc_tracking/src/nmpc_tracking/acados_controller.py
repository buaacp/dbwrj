import ctypes
import os
import time
from typing import Any, Dict, Optional

import numpy as np

from .acados_model import (
    build_acados_model,
    command_bounds,
    command_rate_bounds,
    controller_dimensions,
    stage_weight_matrix,
    terminal_weight_matrix,
)
from .robot_layout import assert_layout_consistency, load_robot_layout


class AcadosUnavailableError(RuntimeError):
    pass


class AcadosSolveError(RuntimeError):
    pass


def preload_acados_link_dependencies() -> None:
    source_dir = os.environ.get("ACADOS_SOURCE_DIR")
    if not source_dir:
        return
    osqp = os.path.join(source_dir, "lib", "libosqp.so")
    if os.path.exists(osqp):
        ctypes.CDLL(osqp, mode=ctypes.RTLD_GLOBAL)


def acados_available() -> bool:
    try:
        preload_acados_link_dependencies()
        import acados_template  # noqa: F401
        return True
    except Exception:
        return False


class AcadosNmpcController:
    def __init__(self, config: Dict[str, Any], build_dir: str = "build/nmpc_acados",
                 verbose: bool = False):
        if not acados_available():
            raise AcadosUnavailableError(
                "acados_template is not available in this Python environment. "
                "Run: source scripts/setup_acados_env.bash && \"$ACADOS_PYTHON\" ..."
            )
        self.config = dict(config)
        self.layout = load_robot_layout(config)
        self.dims = controller_dimensions(config, self.layout)
        self.N = int(config["controller"]["horizon_steps"])
        self.dt = float(config["controller"]["dt"])
        self.tf = self.N * self.dt
        self.build_dir = os.path.abspath(build_dir)
        self.verbose = bool(verbose)
        self.solver = None
        self.ocp = None
        self.reference_window = None
        self.build_count = 0
        self.last_solution = None
        self.command_lower, self.command_upper = command_bounds(config, self.layout)
        self.command_rate_lower, self.command_rate_upper = command_rate_bounds(config, self.layout)
        assert_layout_consistency(
            self.layout, self.state_dim, self.command_dim, self.control_rate_dim)

    @property
    def state_dim(self) -> int:
        return int(self.dims["state_dim"])

    @property
    def command_dim(self) -> int:
        return int(self.dims["command_dim"])

    @property
    def control_rate_dim(self) -> int:
        return int(self.dims["control_rate_dim"])

    @property
    def command_slice(self):
        return self.dims["idx_command"]

    def build(self):
        from acados_template import AcadosOcp, AcadosOcpSolver
        import casadi as ca

        os.makedirs(self.build_dir, exist_ok=True)
        ocp = AcadosOcp()
        model = build_acados_model(self.config, self.layout)
        ocp.model = model
        # 所有 acados 生成代码都放入 build 目录，避免污染源码树。
        ocp.code_export_directory = os.path.join(self.build_dir, "c_generated_code")

        nx = self.state_dim
        nu = self.control_rate_dim
        ocp.solver_options.N_horizon = self.N
        ocp.solver_options.tf = self.tf
        ocp.solver_options.qp_solver = self.config["controller"].get(
            "qp_solver", "PARTIAL_CONDENSING_HPIPM")
        ocp.solver_options.nlp_solver_type = self.config["controller"].get(
            "solver", "SQP_RTI")
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 1
        ocp.solver_options.print_level = 0

        ocp.cost.cost_type = "NONLINEAR_LS"
        # 阶段代价跟踪完整扩展状态 z 和命令变化率 nu。
        # u_c 本身在 z 中，因此 |u_c-u_ref| 和 |nu| 可以同时出现。
        ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)
        ocp.cost.W = stage_weight_matrix(self.config, self.layout)
        ocp.cost.yref = np.zeros(nx + nu)

        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.model.cost_y_expr_e = model.x
        ocp.cost.W_e = terminal_weight_matrix(self.config, self.layout)
        ocp.cost.yref_e = np.zeros(nx)

        ocp.constraints.x0 = np.zeros(nx)
        command_indices = np.arange(self.command_slice.start, self.command_slice.stop, dtype=int)
        # 命令边界作用在状态里的 u_c，命令变化率边界作用在优化输入 nu。
        ocp.constraints.idxbx = command_indices
        ocp.constraints.lbx = self.command_lower.copy()
        ocp.constraints.ubx = self.command_upper.copy()
        ocp.constraints.idxbx_e = command_indices
        ocp.constraints.lbx_e = self.command_lower.copy()
        ocp.constraints.ubx_e = self.command_upper.copy()
        ocp.constraints.idxbu = np.arange(nu, dtype=int)
        ocp.constraints.lbu = self.command_rate_lower.copy()
        ocp.constraints.ubu = self.command_rate_upper.copy()

        json_file = os.path.join(self.build_dir, "nmpc_interface_ocp.json")
        self.solver = AcadosOcpSolver(
            ocp, json_file=json_file, build=True, generate=True, verbose=self.verbose)
        self.ocp = ocp
        self.build_count += 1
        return self

    def set_reference(self, reference_window: np.ndarray) -> None:
        ref = np.asarray(reference_window, dtype=float)
        if ref.shape != (self.N + 1, self.state_dim):
            raise ValueError("reference_window must have shape (%d, %d), got %s" %
                             (self.N + 1, self.state_dim, ref.shape))
        self.reference_window = ref.copy()
        zeros_nu = np.zeros(self.control_rate_dim)
        # reference_window 已经按 NMPC 50 Hz 时间网格展开；
        # 每个 shooting node 设置一个 [z_ref, nu_ref=0]。
        for k in range(self.N):
            self.solver.set(k, "yref", np.r_[ref[k], zeros_nu])
        self.solver.set(self.N, "yref", ref[self.N])

    def set_initial_state(self, z0: np.ndarray) -> None:
        z0 = np.asarray(z0, dtype=float).reshape(self.state_dim)
        if not np.all(np.isfinite(z0)):
            raise ValueError("z0 contains non-finite values")
        self.solver.set(0, "lbx", z0)
        self.solver.set(0, "ubx", z0)
        self.solver.set(0, "x", z0)

    def warm_start(self, previous_solution: Optional[Dict[str, np.ndarray]]) -> None:
        if previous_solution is None:
            return
        xs = np.asarray(previous_solution.get("predicted_states"), dtype=float)
        nus = np.asarray(previous_solution.get("predicted_command_rates"), dtype=float)
        if xs.shape == (self.N + 1, self.state_dim):
            # 标准 receding-horizon warm start：预测序列整体右移，末端复制最后一项。
            shifted_xs = np.vstack((xs[1:], xs[-1:]))
            for k in range(self.N + 1):
                self.solver.set(k, "x", shifted_xs[k])
        if nus.shape == (self.N, self.control_rate_dim):
            shifted_nus = np.vstack((nus[1:], nus[-1:]))
            for k in range(self.N):
                self.solver.set(k, "u", shifted_nus[k])

    def solve(self) -> Dict[str, Any]:
        if self.solver is None:
            self.build()
        if self.reference_window is None:
            raise RuntimeError("set_reference() must be called before solve()")
        start = time.perf_counter()
        status = int(self.solver.solve())
        solve_time = time.perf_counter() - start
        predicted_states = self.get_predicted_states()
        predicted_command_rates = self.get_predicted_controls()
        predicted_commands = predicted_states[1:, self.command_slice]
        first_command = predicted_commands[0].copy()
        cost = self._get_cost()
        result = {
            "status": status,
            "solve_time_s": solve_time,
            "first_command": first_command,
            "predicted_states": predicted_states,
            "predicted_commands": predicted_commands,
            "predicted_command_rates": predicted_command_rates,
            "cost": cost,
        }
        self.last_solution = result
        if status != 0:
            raise AcadosSolveError("acados solve failed with status %d" % status)
        return result

    def get_first_command(self) -> np.ndarray:
        if self.last_solution is None:
            raise RuntimeError("no solution available")
        return np.asarray(self.last_solution["first_command"]).copy()

    def get_predicted_states(self) -> np.ndarray:
        return np.asarray([self.solver.get(k, "x") for k in range(self.N + 1)], dtype=float)

    def get_predicted_controls(self) -> np.ndarray:
        return np.asarray([self.solver.get(k, "u") for k in range(self.N)], dtype=float)

    def _get_cost(self) -> float:
        try:
            return float(self.solver.get_cost())
        except Exception:
            return float("nan")
