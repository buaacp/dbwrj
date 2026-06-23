"""P2.7 whole-body offline bulb pregrasp trajectory optimization."""

from dataclasses import dataclass
from typing import Any, Dict, List, Tuple

import numpy as np
import pinocchio as pin

from .actuation import UamActuation
from .bulb_pregrasp import (
    compute_pregrasp_target, initial_configuration, load_pregrasp_configuration,
    resolve_bulb_pose, solve_terminal_ik)
from .model_loader import UamModel
from .prediction_model import UAMPredictionModel
from .static_trim import StaticTrimSolver


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
        all_costs: List[float] = []
        converged = False; total_iterations = 0
        # Standard shooting nodes cannot couple u[k-1] directly. These passes
        # update a proximal previous-control reference, preserving the original state.
        delta_references = self.trim_references.copy()
        solver = None; problem = None
        for _ in range(2):
            problem = self._build_problem(strategy_config, delta_references)
            solver = crocoddyl.SolverBoxFDDP(problem)
            logger = crocoddyl.CallbackLogger(); solver.setCallbacks([logger])
            converged = bool(solver.solve(
                xs, us, int(self.scenario["max_iterations"]), False, 1e-7))
            total_iterations += len(logger.costs); all_costs.extend(float(v) for v in logger.costs)
            xs = [np.asarray(value).copy() for value in solver.xs]
            us = [np.asarray(value).copy() for value in solver.us]
            controls = np.asarray(us)
            delta_references = np.vstack((controls[0], controls[:-1]))
        controls = np.asarray(solver.us)
        states = np.asarray(problem.rollout(list(solver.us)))
        solver_states = np.asarray(solver.xs)
        rollout_error = max(float(np.linalg.norm(self.robot.state.diff(a, b)))
                            for a, b in zip(states, solver_states))
        return BulbStrategySolution(
            strategy=strategy, report_name=strategy_config["report_name"],
            states=states, controls=controls, trim_references=self.trim_references.copy(),
            reference_states=self.reference_states.copy(), costs=all_costs,
            converged=converged, iterations=total_iterations, rollout_error=rollout_error,
            target_pose=self.target_pose, scenario=self.scenario, ik_report=self.ik_report,
            bulb_diagnostics=self.bulb_diagnostics,
            target_diagnostics=self.target_diagnostics)

    def _build_problem(self, strategy: Dict[str, Any], delta_references: np.ndarray) -> Any:
        import crocoddyl
        running = []
        for index in range(self.steps):
            costs = self._costs(strategy, self.reference_states[index],
                                self.trim_references[index], delta_references[index], False)
            running.append(self.prediction.build_action_model(self.dt, costs))
        terminal_cost = self._costs(
            strategy, self.x_seed, self.trim_references[-1], delta_references[-1], True)
        terminal = self.prediction.build_action_model(0.0, terminal_cost)
        return crocoddyl.ShootingProblem(self.x0, running, terminal)

    def _costs(self, strategy: Dict[str, Any], xref: np.ndarray, trim: np.ndarray,
               previous_control: np.ndarray, terminal: bool) -> Any:
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
        return costs

