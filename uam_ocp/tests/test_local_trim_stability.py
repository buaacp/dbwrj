"""Regression tests for P2.6b manifold LQR and disturbance recovery."""

import unittest

import numpy as np

from uam_ocp.actuation import UamActuation
from uam_ocp.local_lqr import LocalTrimLQR
from uam_ocp.local_stability import (
    build_lqr_weights, select_stability_configurations,
    validate_local_trim_stability)
from uam_ocp.model_loader import MODULE_ROOT, load_uam_model, load_yaml
from uam_ocp.prediction_model import UAMPredictionModel
from uam_ocp.static_trim import StaticTrimSolver


class TestLocalTrimStability(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.robot = load_uam_model()
        cls.actuation = UamActuation(cls.robot)
        cls.prediction = UAMPredictionModel(cls.robot, cls.actuation)
        cls.config = load_yaml(
            MODULE_ROOT / "config" / "local_stability_scenarios.yaml")

    def test_automatic_margin_selection(self) -> None:
        selected, details = select_stability_configurations(
            self.robot, self.actuation, self.config)
        self.assertEqual([item["name"] for item in selected],
                         ["neutral", "fully_extended", "p2_terminal", "left_offset"])
        joint = details["lowest_joint_torque_margin"]
        self.assertEqual(joint["configuration"], "fully_extended")
        self.assertEqual(joint["joint_name"], "left_knuckle_joint")
        self.assertEqual(joint["joint_number_one_based"], 6)
        self.assertEqual(details["lowest_rotor_margin"]["configuration"], "left_offset")

    def test_fully_extended_linear_lqr(self) -> None:
        selected, _ = select_stability_configurations(
            self.robot, self.actuation, self.config)
        entry = next(item for item in selected if item["name"] == "fully_extended")
        q = np.asarray(entry["result"]["q"], dtype=float)
        trim = StaticTrimSolver(self.robot, self.actuation).solve_trim(q)
        x_eq = np.concatenate((q, np.zeros(self.robot.model.nv)))
        Q, R = build_lqr_weights(self.config, self.robot, self.actuation)
        local = LocalTrimLQR(self.prediction)
        design = local.design(x_eq, trim.u_eq,
                              float(self.config["linearization"]["dt_s"]), Q, R)
        self.assertEqual(design.A.shape, (24, 24))
        self.assertEqual(design.B.shape, (24, 10))
        self.assertGreater(np.max(np.abs(design.open_eigenvalues)), 1.0)
        self.assertLess(np.max(np.abs(design.closed_eigenvalues)), 1.0)
        self.assertTrue(design.diagnostics["stabilizable"])

    def test_all_selected_nonlinear_recovery(self) -> None:
        summary = validate_local_trim_stability(
            self.robot, self.actuation, self.prediction)
        self.assertTrue(summary["pass"], summary)
        self.assertEqual(summary["case_count"], 16)
        self.assertLess(summary["maximum_closed_spectral_radius"], 1.0)
        self.assertEqual(summary["maximum_saturation_ratio"], 0.0)


if __name__ == "__main__":
    unittest.main()

