"""Terminal full-body rest cost indexing, configuration, and metrics tests."""

import unittest

import numpy as np

from uam_ocp.bulb_pregrasp import load_pregrasp_configuration
from uam_ocp.model_loader import load_uam_model
from uam_ocp.terminal_rest import (
    metrics_from_state,
    terminal_rest_config,
    velocity_barrier_bounds,
    velocity_weight_vector,
)


class TestTerminalRest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = load_uam_model()
        cls.scenario, _ = load_pregrasp_configuration("nominal_pregrasp")
        cls.rest = terminal_rest_config(cls.scenario)

    def test_velocity_residual_indices_are_tangent_velocity_block(self):
        weights = velocity_weight_vector(self.robot, self.rest)
        nv = self.robot.model.nv
        self.assertEqual(weights.shape[0], self.robot.state.ndx)
        self.assertTrue(np.all(weights[:nv] == 0.0))
        self.assertTrue(np.all(weights[nv:nv + 3] == self.rest["base_linear_velocity_weight"]))
        self.assertTrue(np.all(weights[nv + 3:nv + 6] == self.rest["base_angular_velocity_weight"]))
        self.assertTrue(np.all(weights[nv + 6:] == self.rest["arm_joint_velocity_weight"]))

    def test_barrier_bounds_are_on_velocity_block(self):
        lower, upper = velocity_barrier_bounds(self.robot, self.rest)
        nv = self.robot.model.nv
        self.assertLess(lower[0], -1e8)
        self.assertGreater(upper[0], 1e8)
        np.testing.assert_allclose(upper[nv:nv + 3], self.rest["base_linear_velocity_tolerance_mps"])
        np.testing.assert_allclose(upper[nv + 3:nv + 6], self.rest["base_angular_velocity_tolerance_radps"])
        np.testing.assert_allclose(upper[nv + 6:], self.rest["arm_joint_velocity_tolerance_radps"])

    def test_missing_terminal_rest_config_is_clear_error(self):
        with self.assertRaisesRegex(KeyError, "terminal_rest"):
            terminal_rest_config({})

    def test_batch_synthetic_scenarios_parse(self):
        lateral, _ = load_pregrasp_configuration("lateral_offset_pregrasp")
        vertical, _ = load_pregrasp_configuration("vertical_offset_pregrasp")
        self.assertEqual(lateral["pose_source"], "manual_unvalidated")
        self.assertTrue(lateral["offline_stress_test"])
        self.assertAlmostEqual(lateral["bulb_pose_world"]["position"][1], -0.33)
        self.assertAlmostEqual(vertical["bulb_pose_world"]["position"][2], 0.185)

    def test_terminal_velocity_metrics(self):
        q = self.robot.neutral_configuration({})
        x = np.concatenate((q, np.zeros(self.robot.model.nv)))
        x[self.robot.model.nq:self.robot.model.nq + 3] = [0.03, 0.04, 0.0]
        x[self.robot.model.nq + 3:self.robot.model.nq + 6] = [0.01, 0.02, 0.02]
        x[self.robot.model.nq + 6] = 0.07
        metrics = metrics_from_state(self.robot, x, self.rest)
        self.assertAlmostEqual(metrics["terminal_base_linear_velocity_norm_mps"], 0.05)
        self.assertAlmostEqual(metrics["terminal_max_arm_joint_velocity_radps"], 0.07)
        self.assertTrue(metrics["terminal_rest_pass"])


if __name__ == "__main__":
    unittest.main()
