import os
import sys
import unittest

import numpy as np
import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
UAM = os.path.join(ROOT, "uam_ocp")
for path in [PKG, UAM]:
    if path not in sys.path:
        sys.path.insert(0, path)


class TestPlanApiRegression(unittest.TestCase):
    def test_plan_matches_solve_strategy_contract(self):
        try:
            from uam_ocp.actuation import UamActuation
            from uam_ocp.bulb_pregrasp_planner import BulbPregraspPlanner
            from uam_ocp.model_loader import load_uam_model
            from uam_ocp.prediction_model import UAMPredictionModel
        except Exception as exc:
            self.skipTest("planner dependencies unavailable: %s" % exc)

        robot = load_uam_model()
        actuation = UamActuation(robot)
        prediction = UAMPredictionModel(robot, actuation)
        planner = BulbPregraspPlanner(robot, actuation, prediction)
        old_iterations = planner.scenario["max_iterations"]
        planner.scenario["max_iterations"] = 10
        try:
            original = planner.solve_strategy("whole_body")
            snapshot = planner.plan(
                x0_measured=planner.x0,
                target_pose_override=None,
                previous_solution=None,
                max_iterations=10,
            )
        finally:
            planner.scenario["max_iterations"] = old_iterations

        self.assertEqual(original.states.shape, snapshot.states.shape)
        self.assertEqual(original.controls.shape, snapshot.controls.shape)
        lower, upper = actuation.control_bounds()
        self.assertEqual(lower.shape[0], original.controls.shape[1])
        self.assertEqual(upper.shape[0], original.controls.shape[1])
        self.assertIn("whole_body", planner.scenario["strategies"])
        self.assertIn("terminal_ee_position_error_m", snapshot.terminal_metrics)
        self.assertTrue(np.isfinite(snapshot.terminal_metrics["terminal_ee_position_error_m"]))

        report = {
            "original_states_shape": list(original.states.shape),
            "snapshot_states_shape": list(snapshot.states.shape),
            "original_controls_shape": list(original.controls.shape),
            "snapshot_controls_shape": list(snapshot.controls.shape),
            "control_lower": lower.tolist(),
            "control_upper": upper.tolist(),
            "strategy_present": "whole_body" in planner.scenario["strategies"],
            "snapshot_terminal_metrics": {
                k: float(v) for k, v in snapshot.terminal_metrics.items()
                if isinstance(v, (float, int, np.floating, np.integer))
            },
        }
        out = os.path.join(ROOT, "results", "nmpc_smoke")
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, "plan_api_regression_summary.yaml"), "w") as f:
            yaml.safe_dump(report, f, sort_keys=False)


if __name__ == "__main__":
    unittest.main()
