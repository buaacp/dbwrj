"""Regression tests for the canonical Crocoddyl prediction model."""

import unittest

import numpy as np

from uam_ocp.actuation import UamActuation
from uam_ocp.model_loader import load_uam_model
from uam_ocp.p2_planner import P2Planner
from uam_ocp.prediction_model import UAMPredictionModel
from uam_ocp.prediction_validation import validate_prediction_model


class TestPredictionModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.robot = load_uam_model()
        cls.actuation = UamActuation(cls.robot)
        cls.prediction = UAMPredictionModel(cls.robot, cls.actuation)
        cls.planner = P2Planner(
            cls.robot, cls.actuation, prediction_model=cls.prediction)

    def test_dimensions_bounds_and_hover_definition(self) -> None:
        self.assertEqual((self.prediction.nq, self.prediction.nv), (13, 12))
        self.assertEqual((self.prediction.nx, self.prediction.ndx, self.prediction.nu), (25, 24, 10))
        lower, upper = self.prediction.get_control_bounds()
        self.assertEqual(lower.shape, (10,))
        self.assertEqual(upper.shape, (10,))
        hover = self.prediction.hover_control()
        expected = self.robot.total_mass * np.linalg.norm(self.robot.model.gravity.linear) / 4.0
        np.testing.assert_allclose(hover[:4], expected)
        np.testing.assert_allclose(hover[4:], 0.0)

    def test_p2_consistency_derivatives_and_hover_rollout(self) -> None:
        summary = validate_prediction_model(self.prediction, self.planner)
        self.assertTrue(summary["pass"], summary)


if __name__ == "__main__":
    unittest.main()

