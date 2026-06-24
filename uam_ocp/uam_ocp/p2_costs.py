"""Crocoddyl cost construction for free-flight pre-grasp optimization."""

from typing import Any, Dict

import numpy as np
import pinocchio as pin

from .model_loader import UamModel
from .terminal_rest import add_terminal_rest_costs


def add_weighted_residual(costs: Any, name: str, state: Any, residual: Any,
                          weight: float, component_weights: np.ndarray = None) -> None:
    """Add a scalar-weighted residual, optionally weighting components."""
    import crocoddyl
    if component_weights is None:
        cost = crocoddyl.CostModelResidual(state, residual)
    else:
        activation = crocoddyl.ActivationModelWeightedQuad(
            np.asarray(component_weights, dtype=float))
        cost = crocoddyl.CostModelResidual(state, activation, residual)
    costs.addCost(name, cost, float(weight))


def create_cost_sum(robot: UamModel, nu: int, target_state: np.ndarray,
                    target_pose: pin.SE3, base_rotation: np.ndarray,
                    hover_control: np.ndarray, weights: Dict[str, float],
                    terminal: bool = False, terminal_velocity: bool = True,
                    terminal_rest: Dict[str, float] = None,
                    terminal_rest_window_scale: float = 0.0):
    """Create running or terminal costs without hard-coded state dimensions."""
    import crocoddyl
    state = robot.state
    costs = crocoddyl.CostModelSum(state, nu)
    prefix = "terminal_" if terminal else "running_"
    position = crocoddyl.ResidualModelFrameTranslation(
        state, robot.end_effector_frame_id, target_pose.translation, nu)
    rotation = crocoddyl.ResidualModelFrameRotation(
        state, robot.end_effector_frame_id, target_pose.rotation, nu)
    base_attitude = crocoddyl.ResidualModelFrameRotation(
        state, robot.model.getFrameId(robot.base_frame), base_rotation, nu)
    state_residual = crocoddyl.ResidualModelState(state, target_state, nu)
    add_weighted_residual(costs, "ee_position", state, position, weights[prefix + "ee_position"])
    add_weighted_residual(costs, "ee_rotation", state, rotation, weights[prefix + "ee_rotation"])
    add_weighted_residual(costs, "base_attitude", state, base_attitude, weights[prefix + "base_attitude"])
    add_weighted_residual(costs, "state_regularization", state, state_residual, weights[prefix + "state"])
    lower = np.full(state.ndx, -1e6)
    upper = np.full(state.ndx, 1e6)
    lower[3:5] = -np.deg2rad(30.0)
    upper[3:5] = np.deg2rad(30.0)
    qref = target_state[:robot.model.nq]
    for joint in robot.arm_joints:
        lower[joint.idx_v] = robot.model.lowerPositionLimit[joint.idx_q] - qref[joint.idx_q]
        upper[joint.idx_v] = robot.model.upperPositionLimit[joint.idx_q] - qref[joint.idx_q]
        velocity_limit = robot.model.velocityLimit[joint.idx_v]
        lower[robot.model.nv + joint.idx_v] = -velocity_limit
        upper[robot.model.nv + joint.idx_v] = velocity_limit
    bounds = crocoddyl.ActivationBounds(lower, upper)
    barrier = crocoddyl.ActivationModelQuadraticBarrier(bounds)
    state_limit_cost = crocoddyl.CostModelResidual(state, barrier, state_residual)
    costs.addCost("soft_state_limits", state_limit_cost, float(weights[prefix + "state_limits"]))
    if terminal and terminal_velocity:
        velocity = crocoddyl.ResidualModelFrameVelocity(
            state, robot.end_effector_frame_id, pin.Motion.Zero(),
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED, nu)
        add_weighted_residual(
            costs, "ee_velocity", state, velocity, weights["terminal_ee_velocity"],
            np.ones(6))
    if terminal_rest is not None:
        if terminal:
            add_terminal_rest_costs(
                costs, robot, nu, target_state, terminal_rest, terminal=True, window_scale=1.0)
        elif terminal_rest_window_scale > 0.0:
            add_terminal_rest_costs(
                costs, robot, nu, target_state, terminal_rest,
                terminal=False, window_scale=terminal_rest_window_scale)
    if not terminal:
        control = crocoddyl.ResidualModelControl(state, np.asarray(hover_control, dtype=float))
        add_weighted_residual(costs, "control", state, control, weights["running_control"])
    return costs
