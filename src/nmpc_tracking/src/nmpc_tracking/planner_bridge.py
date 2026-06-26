import time
from typing import Optional, Sequence

import numpy as np

from .trajectory_types import TrajectorySnapshot


def snapshot_from_solution(solution, dt: Optional[float] = None, version: int = 0,
                           arm_joint_names: Optional[Sequence[str]] = None) -> TrajectorySnapshot:
    start = time.time()
    states = np.asarray(solution.states, dtype=float)
    controls = np.asarray(solution.controls, dtype=float)
    dt_value = float(dt if dt is not None else solution.scenario["dt"])
    q_names = ["base_x", "base_y", "base_z", "base_qx", "base_qy", "base_qz", "base_qw"]
    if arm_joint_names is None:
        arm_count = controls.shape[1] - 4
        arm_joint_names = ["joint_%d" % i for i in range(arm_count)]
    q_names += list(arm_joint_names)
    nq = len(q_names)
    total_thrust = np.sum(controls[:, :4], axis=1)
    total_thrust = np.r_[total_thrust, total_thrust[-1]]
    body_rate = states[:, nq + 3:nq + 6]
    return TrajectorySnapshot(
        t0=0.0, dt=dt_value, states=states, controls=controls,
        total_thrust_reference=total_thrust,
        body_rate_reference=body_rate,
        joint_position_reference=states[:, 7:nq],
        joint_velocity_reference=states[:, nq + 6:],
        solve_time=time.time() - start,
        converged=bool(solution.converged),
        terminal_metrics={},
        q_names=q_names,
        v_names=[],
        control_names=[],
        target_translation=np.asarray(solution.target_pose.translation, dtype=float),
        target_rotation=np.asarray(solution.target_pose.rotation, dtype=float),
        version=version,
    )
