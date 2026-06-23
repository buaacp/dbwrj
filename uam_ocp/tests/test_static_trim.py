"""P2.6 regression tests for constrained configuration-dependent trim."""

import unittest

import numpy as np
import pinocchio as pin

from uam_ocp.actuation import UamActuation
from uam_ocp.model_loader import load_uam_model
from uam_ocp.prediction_model import UAMPredictionModel
from uam_ocp.static_trim import (
    JOINT_TORQUE_SATURATION, ROTOR_SATURATION, STRICT_FEASIBLE,
    UNACTUATED_WRENCH, StaticTrimSolver)
from uam_ocp.trim_validation import (
    validate_p2_static_trim_mode, validate_trim_scenarios)


class TestStaticTrim(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.robot = load_uam_model()
        cls.actuation = UamActuation(cls.robot)
        cls.prediction = UAMPredictionModel(cls.robot, cls.actuation)
        cls.solver = StaticTrimSolver(
            cls.robot, cls.actuation, prediction_model=cls.prediction)

    def test_folded_strict_trim_and_aba(self) -> None:
        q = self.robot.neutral_configuration(self.robot.config["initial_arm_configuration"])
        q[:3] = [0.0, 0.0, 1.0]
        result = self.solver.solve_trim(q)
        self.assertEqual(result.status, STRICT_FEASIBLE)
        self.assertTrue(result.strict_feasible)
        self.assertLess(np.linalg.norm(result.generalized_force_residual, ord=np.inf), 1e-9)
        self.assertLess(np.max(np.abs(result.aba_acceleration)), 1e-7)
        self.assertTrue(np.all(result.rotor_margins > 0.0))
        self.assertTrue(np.all(result.joint_torque_margins > 0.0))

    def test_tilted_base_is_not_mislabeled_strict(self) -> None:
        q = self.robot.neutral_configuration()
        q[:3] = [0.0, 0.0, 1.0]
        q[3:7] = pin.Quaternion(pin.rpy.rpyToMatrix(0.2, 0.0, 0.0)).coeffs()
        result = self.solver.solve_trim(q)
        self.assertFalse(result.strict_feasible)
        self.assertEqual(result.status, UNACTUATED_WRENCH)
        self.assertGreater(np.linalg.norm(result.base_force_residual), 1.0)

    def test_saturation_classification(self) -> None:
        q = self.robot.neutral_configuration(self.robot.config["initial_arm_configuration"])
        q[:3] = [0.0, 0.0, 1.0]
        rotor_limited = StaticTrimSolver(self.robot, self.actuation)
        rotor_limited.upper[:self.actuation.n_rotors] = 1.0
        rotor_result = rotor_limited.solve_trim(q)
        self.assertFalse(rotor_result.strict_feasible)
        self.assertEqual(rotor_result.status, ROTOR_SATURATION)

        joint_limited = StaticTrimSolver(self.robot, self.actuation)
        lift_index = self.actuation.n_rotors + self.actuation.joint_names.index(
            "shoulder_lift_joint")
        joint_limited.lower[lift_index] = -0.01
        joint_limited.upper[lift_index] = 0.01
        joint_result = joint_limited.solve_trim(q)
        self.assertFalse(joint_result.strict_feasible)
        self.assertEqual(joint_result.status, JOINT_TORQUE_SATURATION)

    def test_all_configured_scenarios(self) -> None:
        summary = validate_trim_scenarios(
            self.robot, self.actuation, self.solver)
        self.assertTrue(summary["pass"], summary)
        self.assertEqual(summary["strict_feasible_configurations"], 17)
        self.assertEqual(summary["total_configurations"], 17)

    def test_p2_static_trim_mode(self) -> None:
        result = validate_p2_static_trim_mode(
            self.robot, self.actuation, self.prediction)
        self.assertTrue(result["pass"], result)
        self.assertEqual(result["strict_trim_nodes"], result["nodes"])


if __name__ == "__main__":
    unittest.main()
