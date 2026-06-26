from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np
import yaml

LEGACY_6DOF_MESSAGE = (
    "Legacy 6-DoF trajectory is incompatible with iris_arm_planar_4dof. "
    "Regenerate trajectory using lock_shoulder_pan=true."
)


@dataclass(frozen=True)
class RobotLayout:
    """统一描述 Pinocchio、trajectory.npz、YAML 和 NMPC 之间的机器人布局。

    这里是运行时维度和关节顺序的唯一入口。控制器、测试脚本和 replay
    都应从这个结构读取 arm_dof、关节名、索引和质量，避免在多处硬编码。
    """
    base_nq: int
    base_nv: int
    arm_joint_names: List[str]
    arm_dof: int
    q_indices: List[int]
    v_indices: List[int]
    command_indices: List[int]
    lower_position_limits: np.ndarray
    upper_position_limits: np.ndarray
    velocity_limits: np.ndarray
    total_mass_kg: float
    nq: int
    nv: int
    trajectory_arm_dof: int
    trajectory_q_names: List[str]
    trajectory_v_names: List[str]
    trajectory_control_names: List[str]
    mass_override_kg: Optional[float] = None

    @property
    def effective_mass_kg(self) -> float:
        return float(self.mass_override_kg if self.mass_override_kg is not None else self.total_mass_kg)

    def as_dict(self) -> Dict:
        return {
            "base_nq": self.base_nq,
            "base_nv": self.base_nv,
            "nq": self.nq,
            "nv": self.nv,
            "arm_joint_names": list(self.arm_joint_names),
            "arm_dof": self.arm_dof,
            "q_indices": list(self.q_indices),
            "v_indices": list(self.v_indices),
            "command_indices": list(self.command_indices),
            "lower_position_limits": self.lower_position_limits.tolist(),
            "upper_position_limits": self.upper_position_limits.tolist(),
            "velocity_limits": self.velocity_limits.tolist(),
            "total_mass_kg": self.total_mass_kg,
            "effective_mass_kg": self.effective_mass_kg,
            "mass_override_kg": self.mass_override_kg,
            "trajectory_arm_dof": self.trajectory_arm_dof,
            "trajectory_q_names": list(self.trajectory_q_names),
            "trajectory_v_names": list(self.trajectory_v_names),
            "trajectory_control_names": list(self.trajectory_control_names),
        }


def load_yaml_config(path: str) -> Dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _load_trajectory_metadata(path: str) -> Dict:
    data = np.load(path, allow_pickle=False)
    q_names = [str(v) for v in data["q_names"].tolist()]
    v_names = [str(v) for v in data["v_names"].tolist()]
    control_names = [str(v) for v in data["control_names"].tolist()]
    states_shape = tuple(np.asarray(data["states"]).shape)
    controls_shape = tuple(np.asarray(data["controls"]).shape)
    arm_from_q = q_names[7:]
    arm_from_v = [name[:-len("_velocity")] for name in v_names[6:]]
    arm_from_u = [name[:-len("_torque_Nm")] for name in control_names[4:]]
    # 旧 6-DoF 轨迹会把 shoulder_pan/knuckle 当成连续优化变量。
    # planar4 模型必须显式拒绝它，而不是静默截断或补零。
    if len(arm_from_q) == 6 and (
            "shoulder_pan_joint" in arm_from_q or "left_knuckle_joint" in arm_from_q):
        raise ValueError(LEGACY_6DOF_MESSAGE)
    if arm_from_q != arm_from_v or arm_from_q != arm_from_u:
        raise ValueError(
            "trajectory arm name/order mismatch: q=%s v=%s u=%s" %
            (arm_from_q, arm_from_v, arm_from_u))
    inferred_arm = states_shape[1] - len(q_names) - 6
    if inferred_arm != len(arm_from_q):
        raise ValueError("trajectory shape-derived arm DoF %d != name-derived %d" %
                         (inferred_arm, len(arm_from_q)))
    if controls_shape[1] != 4 + len(arm_from_q):
        raise ValueError("trajectory controls shape %s is not 4 + arm DoF %d" %
                         (controls_shape, len(arm_from_q)))
    return {
        "q_names": q_names,
        "v_names": v_names,
        "control_names": control_names,
        "arm_joint_names": arm_from_q,
        "arm_dof": len(arm_from_q),
        "states_shape": states_shape,
        "controls_shape": controls_shape,
        "model_variant": str(data["model_variant"][0]) if "model_variant" in data.files else "",
    }


def load_robot_layout(config: Dict) -> RobotLayout:
    from uam_ocp.model_loader import load_uam_model

    # load_uam_model() 会从 canonical iris_arm xacro 展开 locked shoulder-pan 模型，
    # 再 reduce 出 4-DoF planar arm；这里再和 trajectory/config 做交叉校验。
    robot = load_uam_model()
    traj = _load_trajectory_metadata(config["trajectory"]["offline_npz"])
    arm_joint_names = [joint.name for joint in robot.arm_joints]
    if robot.config.get("model_variant") == "iris_arm_planar_4dof":
        expected = ["shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_roll_joint"]
        if arm_joint_names != expected:
            raise ValueError("canonical reduced arm joints %s != planar4 expected %s" %
                             (arm_joint_names, expected))
    if arm_joint_names != traj["arm_joint_names"]:
        if traj["arm_dof"] != len(arm_joint_names):
            raise ValueError(LEGACY_6DOF_MESSAGE)
        raise ValueError("Pinocchio arm joints %s != trajectory arm joints %s" %
                         (arm_joint_names, traj["arm_joint_names"]))
    config_arm_names = list(config["arm"]["joint_names"])
    if config_arm_names != arm_joint_names:
        raise ValueError("YAML arm.joint_names %s != RobotLayout arm joints %s" %
                         (config_arm_names, arm_joint_names))
    for key in ["velocity_signs", "velocity_accel_limits_radps2"]:
        if len(config["arm"][key]) != len(arm_joint_names):
            raise ValueError("arm.%s length mismatch" % key)
    q_indices = [int(j.idx_q) for j in robot.arm_joints]
    v_indices = [int(j.idx_v) for j in robot.arm_joints]
    lower = np.asarray([j.lower[0] for j in robot.arm_joints], dtype=float)
    upper = np.asarray([j.upper[0] for j in robot.arm_joints], dtype=float)
    velocity = np.asarray([j.velocity_limit[0] for j in robot.arm_joints], dtype=float)
    mass_override = config["vehicle"].get("mass_override_kg")
    if mass_override is not None:
        # 默认使用 Pinocchio/URDF 总质量。override 只用于显式调试，并在运行时报警。
        mass_override = float(mass_override)
        print("WARNING: vehicle.mass_override_kg is set; using %.9g kg instead of model mass %.9g kg" %
              (mass_override, robot.total_mass))
    return RobotLayout(
        base_nq=7,
        base_nv=6,
        arm_joint_names=arm_joint_names,
        arm_dof=len(arm_joint_names),
        q_indices=q_indices,
        v_indices=v_indices,
        command_indices=list(range(4 + len(arm_joint_names))),
        lower_position_limits=lower,
        upper_position_limits=upper,
        velocity_limits=velocity,
        total_mass_kg=float(robot.total_mass),
        nq=int(robot.model.nq),
        nv=int(robot.model.nv),
        trajectory_arm_dof=int(traj["arm_dof"]),
        trajectory_q_names=traj["q_names"],
        trajectory_v_names=traj["v_names"],
        trajectory_control_names=traj["control_names"],
        mass_override_kg=mass_override,
    )


def assert_layout_consistency(layout: RobotLayout, state_dim: int,
                              command_dim: int, rate_dim: int) -> None:
    """启动前强制检查所有维度关系，防止 solver 用错模型或旧轨迹。"""
    if layout.arm_dof != layout.trajectory_arm_dof:
        raise AssertionError("layout arm DoF != trajectory arm DoF")
    if command_dim != 4 + layout.arm_dof:
        raise AssertionError("command_dim %d != 4 + arm_dof %d" %
                             (command_dim, layout.arm_dof))
    expected_state = 16 + 3 * layout.arm_dof
    if state_dim != expected_state:
        raise AssertionError("state_dim %d != 16 + 3*arm_dof %d" %
                             (state_dim, expected_state))
    if rate_dim != command_dim:
        raise AssertionError("rate_dim %d != command_dim %d" % (rate_dim, command_dim))
