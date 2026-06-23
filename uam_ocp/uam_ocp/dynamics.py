"""Pinocchio ABA forward dynamics for physical UAM controls."""

from typing import Tuple

import numpy as np
import pinocchio as pin

from .actuation import UamActuation
from .model_loader import UamModel


def generalized_acceleration(
        robot: UamModel, actuation: UamActuation,
        q: np.ndarray, v: np.ndarray, control: np.ndarray) -> np.ndarray:
    """Return ABA generalized acceleration for one state and physical input."""
    tau = actuation.physical_control_to_generalized_torque(control)
    return np.asarray(pin.aba(robot.model, robot.model.createData(), q, v, tau)).copy()


def state_derivative(
        robot: UamModel, actuation: UamActuation,
        q: np.ndarray, v: np.ndarray, control: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Return tangent configuration velocity and ABA acceleration."""
    return np.asarray(v).copy(), generalized_acceleration(robot, actuation, q, v, control)

