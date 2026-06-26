from typing import Dict, Optional, Tuple

import numpy as np

from .robot_layout import RobotLayout, load_robot_layout


def _require_casadi():
    try:
        import casadi as ca
        return ca
    except Exception as exc:
        raise RuntimeError("casadi is required to build the NMPC interface model") from exc


def _layout(config: Dict, layout: Optional[RobotLayout] = None) -> RobotLayout:
    return layout if layout is not None else load_robot_layout(config)


def controller_dimensions(config: Dict, layout: Optional[RobotLayout] = None) -> Dict[str, int]:
    layout = _layout(config, layout)
    joint_count = layout.arm_dof
    command_dim = 4 + joint_count
    # z = [p_W, v_W, rpy, omega_B, q_a, dq_a, u_c]
    # u_c = [T, p, q, r, dq_a_cmd]，所以 planar4 时 state_dim=28。
    state_dim = 3 + 3 + 3 + 3 + joint_count + joint_count + command_dim
    return {
        "joint_count": joint_count,
        "command_dim": command_dim,
        "control_rate_dim": command_dim,
        "state_dim": state_dim,
        "idx_position": slice(0, 3),
        "idx_velocity": slice(3, 6),
        "idx_attitude": slice(6, 9),
        "idx_body_rate": slice(9, 12),
        "idx_joint_position": slice(12, 12 + joint_count),
        "idx_joint_velocity": slice(12 + joint_count, 12 + 2 * joint_count),
        "idx_command": slice(12 + 2 * joint_count, state_dim),
    }


def build_interface_dynamics(config: Dict, layout: Optional[RobotLayout] = None) -> Dict:
    """Build the continuous-time interface-level NMPC model with CasADi symbols.

    State:
      z = [p_W, v_W, rpy, omega_B, q_a, dq_a, u_c]

    Control:
      nu = dot(u_c)
    """
    ca = _require_casadi()
    layout = _layout(config, layout)
    dims = controller_dimensions(config, layout)
    na = dims["joint_count"]
    nz = dims["state_dim"]
    nu_dim = dims["control_rate_dim"]
    z = ca.SX.sym("z", nz)
    nu = ca.SX.sym("nu", nu_dim)

    i = 0
    p_w = z[i:i + 3]; i += 3
    v_w = z[i:i + 3]; i += 3
    eta = z[i:i + 3]; i += 3
    omega_b = z[i:i + 3]; i += 3
    q_a = z[i:i + na]; i += na
    dq_a = z[i:i + na]; i += na
    u_c = z[i:i + 4 + na]

    thrust = u_c[0]
    omega_cmd = u_c[1:4]
    dq_cmd = u_c[4:4 + na]

    # 接口级模型只描述 PX4 rate/thrust 与关节速度伺服的闭环响应；
    # 不在运行时重新使用旋翼单独推力或关节力矩。
    roll, pitch, yaw = eta[0], eta[1], eta[2]
    cr, sr = ca.cos(roll), ca.sin(roll)
    cp, sp = ca.cos(pitch), ca.sin(pitch)
    cy, sy = ca.cos(yaw), ca.sin(yaw)
    R = ca.vertcat(
        ca.horzcat(cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        ca.horzcat(sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        ca.horzcat(-sp, cp * sr, cp * cr),
    )
    E = ca.vertcat(
        ca.horzcat(1, sr * ca.tan(pitch), cr * ca.tan(pitch)),
        ca.horzcat(0, cr, -sr),
        ca.horzcat(0, sr / cp, cr / cp),
    )

    vehicle = config["vehicle"]
    mass = float(layout.effective_mass_kg)
    gravity = float(vehicle["gravity_mps2"])
    tau_w = float(config["controller"]["tau_body_rate_s"])
    tau_a = float(config["controller"]["tau_arm_servo_s"])

    p_dot = v_w
    v_dot = ca.vertcat(0.0, 0.0, -gravity) + (thrust / mass) * R[:, 2]
    eta_dot = E @ omega_b
    omega_dot = (omega_cmd - omega_b) / tau_w
    q_dot = dq_a
    dq_dot = (dq_cmd - dq_a) / tau_a
    # 把命令 u_c 放进状态，并以 nu=dot(u_c) 为优化输入，
    # 这样可以直接施加真实命令边界和命令变化率边界。
    uc_dot = nu
    z_dot = ca.vertcat(p_dot, v_dot, eta_dot, omega_dot, q_dot, dq_dot, uc_dot)

    return {
        "z": z,
        "nu": nu,
        "z_dot": z_dot,
        "f_expl": ca.Function("nmpc_interface_f_expl", [z, nu], [z_dot]),
        "dimensions": dims,
    }


def build_acados_model(config: Dict, layout: Optional[RobotLayout] = None):
    ca = _require_casadi()
    try:
        from acados_template import AcadosModel
    except Exception as exc:
        raise RuntimeError("acados_template is required to build AcadosModel") from exc

    dynamics = build_interface_dynamics(config, layout)
    z = dynamics["z"]
    nu = dynamics["nu"]
    z_dot = dynamics["z_dot"]
    xdot = ca.SX.sym("z_dot", z.rows())

    model = AcadosModel()
    model.name = "nmpc_interface_model"
    model.x = z
    model.u = nu
    model.xdot = xdot
    model.f_expl_expr = z_dot
    model.f_impl_expr = xdot - z_dot
    return model


def command_bounds(config: Dict, layout: Optional[RobotLayout] = None) -> Tuple[np.ndarray, np.ndarray]:
    layout = _layout(config, layout)
    na = layout.arm_dof
    vehicle = config["vehicle"]
    lower = np.zeros(4 + na)
    upper = np.zeros(4 + na)
    lower[0] = float(vehicle["total_thrust_min_n"])
    upper[0] = float(vehicle["total_thrust_max_n"])
    body_rate_limits = np.asarray(vehicle["body_rate_limit_radps"], dtype=float).reshape(3)
    lower[1:4] = -body_rate_limits
    upper[1:4] = body_rate_limits
    arm_limits = np.asarray(layout.velocity_limits, dtype=float).reshape(na)
    lower[4:] = -arm_limits
    upper[4:] = arm_limits
    return lower, upper


def command_rate_bounds(config: Dict, layout: Optional[RobotLayout] = None) -> Tuple[np.ndarray, np.ndarray]:
    layout = _layout(config, layout)
    na = layout.arm_dof
    vehicle = config["vehicle"]
    limits = np.zeros(4 + na)
    limits[0] = float(vehicle["total_thrust_rate_limit_nps"])
    limits[1:4] = np.asarray(vehicle["body_rate_accel_limit_radps2"], dtype=float).reshape(3)
    limits[4:] = np.asarray(config["arm"]["velocity_accel_limits_radps2"], dtype=float).reshape(na)
    return -limits, limits


def stage_weight_matrix(config: Dict, layout: Optional[RobotLayout] = None) -> np.ndarray:
    layout = _layout(config, layout)
    dims = controller_dimensions(config, layout)
    na = dims["joint_count"]
    sigma = config["controller"]["sigma"]
    mult = config["controller"].get("weight_multipliers", {})
    # 权重采用“允许误差 sigma”的归一化形式；扫描 Qp/Qv/Qp_terminal 时
    # 只改 multiplier，不改物理单位或状态定义。
    state_sigmas = np.r_[
        np.full(3, float(sigma["position_m"])),
        np.full(3, float(sigma["velocity_mps"])),
        np.full(3, float(sigma["attitude_rad"])),
        np.full(3, float(sigma["body_rate_radps"])),
        np.full(na, float(sigma["joint_position_rad"])),
        np.full(na, float(sigma["joint_velocity_radps"])),
        np.full(dims["command_dim"], float(sigma["command"])),
    ]
    control_sigmas = np.full(dims["control_rate_dim"], float(sigma["command_rate"]))
    weights = np.r_[1.0 / (state_sigmas ** 2), 1.0 / (control_sigmas ** 2)]
    weights[0:3] *= float(mult.get("position", 1.0))
    weights[3:6] *= float(mult.get("velocity", 1.0))
    weights[6:9] *= float(mult.get("attitude", 1.0))
    weights[9:12] *= float(mult.get("body_rate", 1.0))
    weights[12:12 + na] *= float(mult.get("joint_position", 1.0))
    weights[12 + na:12 + 2 * na] *= float(mult.get("joint_velocity", 1.0))
    weights[dims["state_dim"]:dims["state_dim"] + dims["command_dim"]] *= float(mult.get("command", 1.0))
    weights[dims["state_dim"] + dims["command_dim"]:] *= float(mult.get("command_rate", 1.0))
    return np.diag(weights)


def terminal_weight_matrix(config: Dict, layout: Optional[RobotLayout] = None) -> np.ndarray:
    layout = _layout(config, layout)
    dims = controller_dimensions(config, layout)
    na = dims["joint_count"]
    sigma = config["controller"]["sigma"]
    mult = config["controller"].get("weight_multipliers", {})
    state_sigmas = np.r_[
        np.full(3, float(sigma["position_m"])),
        np.full(3, float(sigma["velocity_mps"])),
        np.full(3, float(sigma["attitude_rad"])),
        np.full(3, float(sigma["body_rate_radps"])),
        np.full(na, float(sigma["joint_position_rad"])),
        np.full(na, float(sigma["joint_velocity_radps"])),
        np.full(dims["command_dim"], float(sigma["command"])),
    ]
    weights = 1.0 / (state_sigmas ** 2)
    weights[0:3] *= float(mult.get("terminal_position", mult.get("position", 1.0)))
    weights[3:6] *= float(mult.get("terminal_velocity", mult.get("velocity", 1.0)))
    weights[6:9] *= float(mult.get("terminal_attitude", mult.get("attitude", 1.0)))
    weights[9:12] *= float(mult.get("terminal_body_rate", mult.get("body_rate", 1.0)))
    weights[12:12 + na] *= float(mult.get("terminal_joint_position", mult.get("joint_position", 1.0)))
    weights[12 + na:12 + 2 * na] *= float(mult.get("terminal_joint_velocity", mult.get("joint_velocity", 1.0)))
    return np.diag(weights)


def hover_command(config: Dict, layout: Optional[RobotLayout] = None) -> np.ndarray:
    layout = _layout(config, layout)
    dims = controller_dimensions(config, layout)
    cmd = np.zeros(dims["command_dim"])
    cmd[0] = float(layout.effective_mass_kg) * float(config["vehicle"]["gravity_mps2"])
    return cmd
