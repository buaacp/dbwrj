"""Canonical Crocoddyl full-body prediction model for UAM optimization."""

from typing import Any, Optional, Tuple

import numpy as np

from .actuation import UamActuation
from .model_loader import UamModel


class UAMPredictionModel:
    """Nonlinear full-body prediction model for UAM optimization.

    State:
        x = [q, v], with q on the free-flyer multibody manifold.

    Control:
        u = [T1, T2, T3, T4, tau_arm_1, ..., tau_arm_6].

    The discrete transition and its derivatives are both evaluated by
    Crocoddyl's IntegratedActionModelEuler. No separate hand-written
    integration path is used.
    """

    def __init__(self, robot: UamModel, actuation: Optional[UamActuation] = None) -> None:
        self.robot = robot
        self.state = robot.state
        self.actuation = actuation or UamActuation(robot)
        self.crocoddyl_actuation = self.actuation.crocoddyl_model()
        self.nq = int(robot.model.nq)
        self.nv = int(robot.model.nv)
        self.nx = int(self.state.nx)
        self.ndx = int(self.state.ndx)
        self.nu = int(self.actuation.nu)

    def build_prediction_action_model(self, dt: float) -> Any:
        """Build a zero-cost Crocoddyl action model for one prediction step."""
        import crocoddyl
        costs = crocoddyl.CostModelSum(self.state, self.nu)
        return self.build_action_model(dt, costs)

    def build_action_model(self, dt: float, costs: Any) -> Any:
        """Build the canonical dynamics with caller-supplied optimization costs."""
        import crocoddyl
        dt = float(dt)
        if dt < 0.0:
            raise ValueError(f"dt must be nonnegative, got {dt}")
        differential = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self.state, self.crocoddyl_actuation, costs)
        integrated = crocoddyl.IntegratedActionModelEuler(differential, dt)
        lower, upper = self.get_control_bounds()
        integrated.u_lb = lower
        integrated.u_ub = upper
        return integrated

    def step(self, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        """Evaluate x_next = f_pred(x, u, dt) through Crocoddyl calc()."""
        x = self._state_vector(x)
        u = self._control_vector(u)
        action = self.build_prediction_action_model(dt)
        data = action.createData()
        action.calc(data, x, u)
        return np.asarray(data.xnext, dtype=float).copy()

    def rollout(self, x0: np.ndarray, controls: np.ndarray, dt: float) -> np.ndarray:
        """Roll out a control sequence using one shared Crocoddyl action model."""
        x = self._state_vector(x0)
        controls = np.asarray(controls, dtype=float)
        if controls.ndim != 2 or controls.shape[1] != self.nu:
            raise ValueError(f"controls must have shape (N, {self.nu}), got {controls.shape}")
        action = self.build_prediction_action_model(dt)
        data = action.createData()
        states = [x.copy()]
        for control in controls:
            action.calc(data, x, self._control_vector(control))
            x = np.asarray(data.xnext, dtype=float).copy()
            states.append(x)
        return np.asarray(states)

    def linearize(self, x: np.ndarray, u: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """Return manifold-tangent discrete derivatives from Crocoddyl calcDiff()."""
        x = self._state_vector(x)
        u = self._control_vector(u)
        action = self.build_prediction_action_model(dt)
        data = action.createData()
        action.calc(data, x, u)
        action.calcDiff(data, x, u)
        return (np.asarray(data.Fx, dtype=float).copy(),
                np.asarray(data.Fu, dtype=float).copy())

    def get_control_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return configured rotor-thrust and arm-torque bounds."""
        lower, upper = self.actuation.control_bounds()
        return lower.copy(), upper.copy()

    def hover_control(self) -> np.ndarray:
        """Return equal mg/4 rotor thrust with zero arm joint torque."""
        gravity = float(np.linalg.norm(self.robot.model.gravity.linear))
        return self.actuation.equal_hover_control(gravity)

    def static_trim_control(self, q: np.ndarray) -> np.ndarray:
        """Return full-model static trim for diagnostics and optimizer warm starts."""
        return self.actuation.gravity_compensated_hover_control(
            np.asarray(q, dtype=float).reshape(self.nq))

    def _state_vector(self, x: np.ndarray) -> np.ndarray:
        value = np.asarray(x, dtype=float).reshape(-1)
        if value.size != self.nx or not np.all(np.isfinite(value)):
            raise ValueError(f"state must contain {self.nx} finite values")
        return value

    def _control_vector(self, u: np.ndarray) -> np.ndarray:
        value = np.asarray(u, dtype=float).reshape(-1)
        if value.size != self.nu or not np.all(np.isfinite(value)):
            raise ValueError(f"control must contain {self.nu} finite values")
        return value
