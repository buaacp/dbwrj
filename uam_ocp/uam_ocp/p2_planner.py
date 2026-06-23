"""Box-FDDP free-flight pre-grasp planner."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import pinocchio as pin

from .actuation import UamActuation
from .model_loader import MODULE_ROOT, UamModel, load_yaml
from .p2_costs import create_cost_sum
from .prediction_model import UAMPredictionModel
from .static_trim import StaticTrimSolver


@dataclass
class P2Solution:
    """Solver output and consistency diagnostics."""

    states: np.ndarray
    controls: np.ndarray
    solver_states: np.ndarray
    costs: List[float]
    iterations: int
    converged: bool
    rollout_error: float
    target_pose: pin.SE3
    target_state: np.ndarray
    scenario: Dict[str, Any]


class P2Planner:
    """Construct and solve the configured full-state Crocoddyl problem."""

    def __init__(self, robot: UamModel, actuation: UamActuation,
                 scenarios_path: Optional[Path] = None,
                 prediction_model: Optional[UAMPredictionModel] = None):
        self.robot = robot
        self.actuation = actuation
        self.scenarios = load_yaml(
            scenarios_path or MODULE_ROOT / "config" / "p2_scenarios.yaml")
        self.prediction_model = prediction_model or UAMPredictionModel(robot, actuation)
        if self.prediction_model.robot is not robot or self.prediction_model.actuation is not actuation:
            raise ValueError("P2Planner and prediction model must share robot and actuation instances")
        self.native_actuation = self.prediction_model.crocoddyl_actuation
        self.last_control_references = None
        self.last_trim_results = []

    def initial_state(self, scenario: Dict[str, Any]) -> np.ndarray:
        """Build normalized q and zero generalized velocity from named YAML values."""
        q = self.robot.neutral_configuration(scenario["initial_arm_configuration"])
        q[:3] = np.asarray(scenario["initial_base_position"], dtype=float)
        q[3:7] = np.asarray(scenario["initial_base_quaternion_xyzw"], dtype=float)
        q = pin.normalize(self.robot.model, q)
        return np.concatenate((q, np.zeros(self.robot.model.nv)))

    @staticmethod
    def target_pose(scenario: Dict[str, Any]) -> pin.SE3:
        """Convert configured world EE pose into Pinocchio SE3."""
        quaternion = pin.Quaternion(np.asarray(scenario["pregrasp_ee_quaternion_xyzw"], dtype=float))
        quaternion.normalize()
        return pin.SE3(quaternion.matrix(), np.asarray(scenario["pregrasp_ee_position_world"], dtype=float))

    def target_state(self, x0: np.ndarray, target_pose: pin.SE3) -> np.ndarray:
        """Use a rigid base translation to create a reachable target-state seed."""
        q = x0[:self.robot.model.nq].copy()
        data = self.robot.model.createData()
        pin.forwardKinematics(self.robot.model, data, q)
        pin.updateFramePlacements(self.robot.model, data)
        current = data.oMf[self.robot.end_effector_frame_id]
        q[:3] += target_pose.translation - current.translation
        return np.concatenate((pin.normalize(self.robot.model, q), np.zeros(self.robot.model.nv)))

    def build_problem(self, scenario_name: str = "pregrasp"):
        """Build StateMultibody -> FreeFwdDynamics -> Euler -> ShootingProblem."""
        import crocoddyl
        scenario = self.scenarios[scenario_name]
        x0 = self.initial_state(scenario)
        target_pose = self.target_pose(scenario)
        xref = self.target_state(x0, target_pose)
        dt = float(scenario["dt_s"])
        steps = int(round(float(scenario["horizon_s"]) / dt))
        reference_difference = self.robot.state.diff(x0, xref)
        reference_states = [
            self.robot.state.integrate(x0, (index / steps) * reference_difference)
            for index in range(steps + 1)
        ]
        mode = str(scenario.get("control_reference_mode", "fixed_initial_trim"))
        self.last_trim_results = []
        if mode == "fixed_initial_trim":
            hover = self.actuation.gravity_compensated_hover_control(x0[:self.robot.model.nq])
            control_references = np.tile(hover, (steps, 1))
        elif mode == "static_trim":
            trim_solver = StaticTrimSolver(
                self.robot, self.actuation, prediction_model=self.prediction_model)
            values = []
            for index, reference_state in enumerate(reference_states[:-1]):
                result = trim_solver.solve_trim(reference_state[:self.robot.model.nq])
                self.last_trim_results.append(result)
                if not result.strict_feasible:
                    raise RuntimeError(
                        f"P2 static trim failed at node {index}: status={result.status}, "
                        f"residual={np.linalg.norm(result.generalized_force_residual, ord=np.inf):.3e}")
                values.append(result.u_eq)
            control_references = np.asarray(values)
            hover = control_references
        else:
            raise ValueError(
                f"Unknown control_reference_mode {mode!r}; expected fixed_initial_trim or static_trim")
        self.last_control_references = control_references.copy()
        weights = scenario["weights"]
        if mode == "fixed_initial_trim":
            running_cost = create_cost_sum(
                self.robot, self.actuation.nu, xref, target_pose, np.eye(3),
                control_references[0], weights, terminal=False)
            running_models = [self.prediction_model.build_action_model(dt, running_cost)] * steps
        else:
            running_models = []
            for control_reference in control_references:
                running_cost = create_cost_sum(
                    self.robot, self.actuation.nu, xref, target_pose, np.eye(3),
                    control_reference, weights, terminal=False)
                running_models.append(
                    self.prediction_model.build_action_model(dt, running_cost))
        terminal_cost = create_cost_sum(
            self.robot, self.actuation.nu, xref, target_pose, np.eye(3),
            control_references[-1], weights,
            terminal=True, terminal_velocity=bool(scenario["terminal_ee_velocity"]))
        terminal = self.prediction_model.build_action_model(0.0, terminal_cost)
        problem = crocoddyl.ShootingProblem(x0, running_models, terminal)
        return problem, scenario, target_pose, xref, hover

    def warm_start(self, problem: Any, target_state: np.ndarray,
                   hover: np.ndarray):
        """Interpolate on StateMultibody and initialize controls with static trim."""
        difference = self.robot.state.diff(problem.x0, target_state)
        xs = [self.robot.state.integrate(problem.x0, (index / problem.T) * difference)
              for index in range(problem.T + 1)]
        hover = np.asarray(hover, dtype=float)
        if hover.ndim == 1:
            us = [hover.copy() for _ in range(problem.T)]
        elif hover.shape == (problem.T, self.actuation.nu):
            us = [item.copy() for item in hover]
        else:
            raise ValueError(
                f"Control warm start must have shape ({self.actuation.nu},) or "
                f"({problem.T}, {self.actuation.nu}), got {hover.shape}")
        return xs, us

    def solve(self, scenario_name: str = "pregrasp") -> P2Solution:
        """Run SolverBoxFDDP and return a dynamically rolled-out trajectory."""
        import crocoddyl
        problem, scenario, target_pose, xref, hover = self.build_problem(scenario_name)
        xs, us = self.warm_start(problem, xref, hover)
        solver = crocoddyl.SolverBoxFDDP(problem)
        logger = crocoddyl.CallbackLogger()
        solver.setCallbacks([logger])
        converged = bool(solver.solve(xs, us, int(scenario["max_iterations"]), False, 1e-7))
        solver_states = np.asarray(solver.xs)
        controls = np.asarray(solver.us)
        rollout = np.asarray(problem.rollout(list(solver.us)))
        errors = [np.linalg.norm(self.robot.state.diff(rollout[i], solver_states[i]))
                  for i in range(problem.T + 1)]
        return P2Solution(
            states=rollout, controls=controls, solver_states=solver_states,
            costs=[float(value) for value in logger.costs], iterations=len(logger.costs),
            converged=converged, rollout_error=float(max(errors)), target_pose=target_pose,
            target_state=xref, scenario=scenario)
