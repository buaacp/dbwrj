"""P0 regression tests for the generated floating-base model."""

import unittest

from uam_ocp.model_loader import load_uam_model
from uam_ocp.model_validation import validate_model


class TestP0Model(unittest.TestCase):
    def test_dimensions_and_dynamics(self) -> None:
        robot = load_uam_model()
        summary = validate_model(robot)
        self.assertTrue(summary["pass"], summary["checks"])
        self.assertEqual(robot.model.nq, 7 + robot.n_arm)
        self.assertEqual(robot.model.nv, 6 + robot.n_arm)
        self.assertEqual(robot.n_arm, 6)


if __name__ == "__main__":
    unittest.main()

