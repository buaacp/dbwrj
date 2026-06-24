"""P2.7 whole-body offline bulb pregrasp trajectory optimization."""

from dataclasses import dataclass
from typing import Any, Dict, List, Tuple

import numpy as np
import pinocchio as pin
import crocoddyl

from .actuation import UamActuation
from .bulb_pregrasp import (
    compute_pregrasp_target, initial_configuration, load_pregrasp_configuration,
    resolve_bulb_pose, solve_terminal_ik)
from .model_loader import UamModel
from .prediction_model import UAMPredictionModel
from .static_trim import StaticTrimSolver
from .terminal_rest import add_terminal_rest_costs, terminal_rest_config


@dataclass
class BulbStrategySolution:
    strategy: str
    report_name: str
    states: np.ndarray
    controls: np.ndarray
    trim_references: np.ndarray
    reference_states: np.ndarray
    costs: List[float]
    converged: bool
    iterations: int
    costs_pass_1: List[float]
    costs_pass_2: List[float]
    iterations_pass_1: int
    iterations_pass_2: int
    converged_pass_1: bool
    converged_pass_2: bool
    total_iterations: int
    diagnostics_pass_1: Dict[str, List[float]]
    diagnostics_pass_2: Dict[str, List[float]]
    dynamics_gap_penalty_weight: float
    rollout_error: float
    target_pose: pin.SE3
    scenario: Dict[str, Any]
    ik_report: Dict[str, Any]
    bulb_diagnostics: Dict[str, Any]
    target_diagnostics: Dict[str, Any]


class BulbPregraspPlanner:
    """Build and solve the three soft-strategy P2.7 BoxFDDP problems."""

    def __init__(self, robot: UamModel, actuation: UamActuation,
                 prediction: UAMPredictionModel,
                 scenario_name: str = "scene_bulb_pregrasp") -> None:
        self.robot = robot; self.actuation = actuation; self.prediction = prediction
        self.scenario, self.task_joints = load_pregrasp_configuration(scenario_name)
        self.bulb_pose, self.bulb_diagnostics = resolve_bulb_pose(self.scenario)
        self.target_pose, self.target_diagnostics = compute_pregrasp_target(
            self.scenario, self.bulb_pose)
        self.q0 = initial_configuration(robot, self.scenario)
        self.q_seed, self.ik_report = solve_terminal_ik(
            robot, self.scenario, self.task_joints, self.target_pose)
        self.x0 = np.concatenate((self.q0, np.zeros(robot.model.nv)))
        self.x_seed = np.concatenate((self.q_seed, np.zeros(robot.model.nv)))
        self.steps = int(self.scenario["horizon_steps"]); self.dt = float(self.scenario["dt"])
        self.terminal_rest = terminal_rest_config(self.scenario)
        self.dynamics_gap_penalty_weight = float(
            self.scenario.get("diagnostics", {}).get("dynamics_gap_penalty_weight", 1.0e5))
        difference = robot.state.diff(self.x0, self.x_seed)
        self.reference_states = np.asarray([
            robot.state.integrate(self.x0, (index / self.steps) * difference)
            for index in range(self.steps + 1)])
        trim_solver = StaticTrimSolver(robot, actuation, prediction_model=prediction)
        trims = []
        for index, state in enumerate(self.reference_states[:-1]):
            result = trim_solver.solve_trim(state[:robot.model.nq])
            if not result.strict_feasible:
                raise RuntimeError(f"P2.7 strict trim failed at node {index}: {result.status}")
            trims.append(result.u_eq)
        self.trim_references = np.asarray(trims)

    def solve_strategy(self, strategy: str) -> BulbStrategySolution:
        """Solve one soft coordination strategy with two delta-u proximal passes."""
        import crocoddyl
        if strategy not in self.scenario["strategies"]:
            raise KeyError(strategy)
        strategy_config = self.scenario["strategies"][strategy]
        xs = [state.copy() for state in self.reference_states]
        us = [value.copy() for value in self.trim_references]
        diagnostics_pass_1: Dict[str, List[float]] = {}
        diagnostics_pass_2: Dict[str, List[float]] = {}
        converged_pass_1 = False
        converged_pass_2 = False
        # Standard shooting nodes cannot couple u[k-1] directly. These passes
        # update a proximal previous-control reference, preserving the original state.
        delta_references = self.trim_references.copy()
        solver = None; problem = None
        for pass_index in range(2):
            problem = self._build_problem(strategy_config, delta_references)
            solver = crocoddyl.SolverBoxFDDP(problem)
            diagnostics = self._new_diagnostic_history()
            self._append_diagnostics(diagnostics, problem, xs, us)
            callback = _BulbDiagnosticCallback(self, problem, diagnostics)
            logger = crocoddyl.CallbackLogger(); solver.setCallbacks([logger, callback])
            converged = bool(solver.solve(
                xs, us, int(self.scenario["max_iterations"]), False, 1e-7))
            if pass_index == 0:
                diagnostics_pass_1 = diagnostics
                converged_pass_1 = converged
            else:
                diagnostics_pass_2 = diagnostics
                converged_pass_2 = converged
            xs = [np.asarray(value).copy() for value in solver.xs]
            us = [np.asarray(value).copy() for value in solver.us]
            controls = np.asarray(us)
            delta_references = np.vstack((controls[0], controls[:-1]))
        costs_pass_1 = diagnostics_pass_1["diagnostic_total_cost"]
        costs_pass_2 = diagnostics_pass_2["diagnostic_total_cost"]
        iterations_pass_1 = max(0, len(costs_pass_1) - 1)
        iterations_pass_2 = max(0, len(costs_pass_2) - 1)
        total_iterations = iterations_pass_1 + iterations_pass_2
        controls = np.asarray(solver.us)
        states = np.asarray(problem.rollout(list(solver.us)))
        solver_states = np.asarray(solver.xs)
        rollout_error = max(float(np.linalg.norm(self.robot.state.diff(a, b)))
                            for a, b in zip(states, solver_states))
        return BulbStrategySolution(
            strategy=strategy, report_name=strategy_config["report_name"],
            states=states, controls=controls, trim_references=self.trim_references.copy(),
            reference_states=self.reference_states.copy(), costs=costs_pass_2,
            converged=converged_pass_2, iterations=iterations_pass_2,
            costs_pass_1=costs_pass_1, costs_pass_2=costs_pass_2,
            iterations_pass_1=iterations_pass_1, iterations_pass_2=iterations_pass_2,
            converged_pass_1=converged_pass_1, converged_pass_2=converged_pass_2,
            total_iterations=total_iterations,
            diagnostics_pass_1=diagnostics_pass_1,
            diagnostics_pass_2=diagnostics_pass_2,
            dynamics_gap_penalty_weight=self.dynamics_gap_penalty_weight,
            rollout_error=rollout_error,
            target_pose=self.target_pose, scenario=self.scenario, ik_report=self.ik_report,
            bulb_diagnostics=self.bulb_diagnostics,
            target_diagnostics=self.target_diagnostics)

    def _new_diagnostic_history(self) -> Dict[str, List[float]]:
        return {
            "objective_cost": [],
            "dynamics_gap_sum_squares": [],
            "dynamics_gap_max": [],
            "dynamics_gap_penalty": [],
            "diagnostic_total_cost": [],
            "terminal_ee_position_error_m": [],
            "terminal_ee_orientation_error_rad": [],
            "terminal_base_linear_velocity_norm_mps": [],
            "terminal_base_angular_velocity_norm_radps": [],
            "terminal_max_arm_joint_velocity_radps": [],
            "terminal_full_body_velocity_norm": [],
        }

    def _append_diagnostics(self, history: Dict[str, List[float]], problem: Any,
                            xs: List[np.ndarray], us: List[np.ndarray]) -> None:
        values = self._evaluate_diagnostics(problem, xs, us)
        for key, value in values.items():
            history[key].append(float(value))

    def _evaluate_diagnostics(self, problem: Any, xs: List[np.ndarray],
                              us: List[np.ndarray]) -> Dict[str, float]:
        """Evaluate objective, discrete dynamics defects, and terminal errors."""
        objective = 0.0
        max_gap = 0.0
        gap_squares = 0.0
        for index, (model, data, control) in enumerate(
                zip(problem.runningModels, problem.runningDatas, us)):
            model.calc(data, xs[index], control)
            objective += float(data.cost)
            gap = float(np.linalg.norm(self.robot.state.diff(data.xnext, xs[index + 1])))
            max_gap = max(max_gap, gap)
            gap_squares += gap * gap
        problem.terminalModel.calc(problem.terminalData, xs[-1])
        objective += float(problem.terminalData.cost)

        q = np.asarray(xs[-1])[:self.robot.model.nq]
        data = self.robot.model.createData()
        pin.forwardKinematics(self.robot.model, data, q)
        pin.updateFramePlacements(self.robot.model, data)
        ee = data.oMf[self.robot.end_effector_frame_id]
        rest_metrics = self._terminal_rest_metrics(xs[-1])
        gap_penalty = self.dynamics_gap_penalty_weight * gap_squares
        return {
            "objective_cost": objective,
            "dynamics_gap_sum_squares": gap_squares,
            "dynamics_gap_max": max_gap,
            "dynamics_gap_penalty": gap_penalty,
            "diagnostic_total_cost": objective + gap_penalty,
            "terminal_ee_position_error_m": float(
                np.linalg.norm(ee.translation - self.target_pose.translation)),
            "terminal_ee_orientation_error_rad": float(
                np.linalg.norm(pin.log3(self.target_pose.rotation.T @ ee.rotation))),
            "terminal_base_linear_velocity_norm_mps": rest_metrics[
                "terminal_base_linear_velocity_norm_mps"],
            "terminal_base_angular_velocity_norm_radps": rest_metrics[
                "terminal_base_angular_velocity_norm_radps"],
            "terminal_max_arm_joint_velocity_radps": rest_metrics[
                "terminal_max_arm_joint_velocity_radps"],
            "terminal_full_body_velocity_norm": rest_metrics[
                "terminal_full_body_velocity_norm"],
        }

    def _terminal_rest_metrics(self, state: np.ndarray) -> Dict[str, float]:
        v = np.asarray(state, dtype=float)[self.robot.model.nq:]
        return {
            "terminal_base_linear_velocity_norm_mps": float(np.linalg.norm(v[:3])),
            "terminal_base_angular_velocity_norm_radps": float(np.linalg.norm(v[3:6])),
            "terminal_max_arm_joint_velocity_radps": (
                float(np.max(np.abs(v[6:]))) if v.size > 6 else 0.0),
            "terminal_full_body_velocity_norm": float(np.linalg.norm(v)),
        }

    def _build_problem(self, strategy: Dict[str, Any], delta_references: np.ndarray) -> Any:
        import crocoddyl
        running = []
        for index in range(self.steps):
            scale = self._terminal_rest_window_scale(index)
            costs = self._costs(strategy, self.reference_states[index],
                                self.trim_references[index], delta_references[index], False,
                                rest_window_scale=scale)
            running.append(self.prediction.build_action_model(self.dt, costs))
        terminal_cost = self._costs(
            strategy, self.x_seed, self.trim_references[-1], delta_references[-1], True)
        terminal = self.prediction.build_action_model(0.0, terminal_cost)
        return crocoddyl.ShootingProblem(self.x0, running, terminal)

    def _terminal_rest_window_scale(self, index: int) -> float:
        window = max(0, int(self.terminal_rest["window_steps"]))
        if window <= 0:
            return 0.0
        first = max(0, self.steps - window)
        if int(index) < first:
            return 0.0
        return float(index - first + 1) / float(max(1, self.steps - first))

    def _costs(self, strategy: Dict[str, Any], xref: np.ndarray, trim: np.ndarray,
               previous_control: np.ndarray, terminal: bool,
               rest_window_scale: float = 0.0) -> Any:
        import crocoddyl
        state = self.robot.state; nu = self.actuation.nu
        costs = crocoddyl.CostModelSum(state, nu); common = self.scenario["common_weights"]
        def add(name: str, residual: Any, weight: float, activation: Any = None) -> None:
            cost = (crocoddyl.CostModelResidual(state, residual) if activation is None
                    else crocoddyl.CostModelResidual(state, activation, residual))
            costs.addCost(name, cost, float(weight))
        pos = crocoddyl.ResidualModelFrameTranslation(
            state, self.robot.end_effector_frame_id, self.target_pose.translation, nu)
        rot = crocoddyl.ResidualModelFrameRotation(
            state, self.robot.end_effector_frame_id, self.target_pose.rotation, nu)
        vel = crocoddyl.ResidualModelFrameVelocity(
            state, self.robot.end_effector_frame_id, pin.Motion.Zero(),
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED, nu)
        factor = 1.0 if not terminal else 1.0
        add("ee_position", pos, common["terminal_ee_position"] if terminal else common["running_ee_position"])
        add("ee_rotation", rot, common["terminal_ee_rotation"] if terminal else common["running_ee_rotation"])
        add("ee_velocity", vel, common["terminal_ee_velocity"] if terminal else common["running_ee_velocity"])
        residual_state = crocoddyl.ResidualModelState(state, xref, nu)
        weights = np.ones(state.ndx)
        weights[:3] = strategy["base_position"]; weights[3:6] = strategy["base_attitude"]
        weights[6:12] = strategy["arm_position"]; weights[12:] = strategy["velocity"]
        # Keep the independent gripper/knuckle at its explicit open reference.
        knuckle = self.robot.model.joints[self.robot.model.getJointId("left_knuckle_joint")].idx_v
        weights[knuckle] = max(weights[knuckle], 200.0)
        add("strategy_state", residual_state, factor,
            crocoddyl.ActivationModelWeightedQuad(weights))
        if not terminal:
            add("trim_control", crocoddyl.ResidualModelControl(state, trim), common["running_control"])
            add("delta_u_proximal", crocoddyl.ResidualModelControl(state, previous_control), common["delta_u"])
            if rest_window_scale > 0.0:
                add_terminal_rest_costs(
                    costs, self.robot, nu, xref, self.terminal_rest,
                    terminal=False, window_scale=rest_window_scale)
        else:
            add_terminal_rest_costs(
                costs, self.robot, nu, xref, self.terminal_rest,
                terminal=True, window_scale=1.0)
        return costs


class _BulbDiagnosticCallback(crocoddyl.CallbackAbstract):
    """Collect per-iteration diagnostics from the current solver trajectory."""

    def __init__(self, planner: BulbPregraspPlanner, problem: Any,
                 history: Dict[str, List[float]]) -> None:
        crocoddyl.CallbackAbstract.__init__(self)
        self.planner = planner
        self.problem = problem
        self.history = history

    def __call__(self, solver: Any) -> None:
        self.planner._append_diagnostics(
            self.history, self.problem,
            [np.asarray(value) for value in solver.xs],
            [np.asarray(value) for value in solver.us])
