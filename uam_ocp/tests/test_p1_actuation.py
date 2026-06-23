"""P1 regression tests for mixed actuation and coupled ABA dynamics."""

import unittest

import numpy as np

from uam_ocp.actuation import UamActuation
from uam_ocp.dynamics import generalized_acceleration
from uam_ocp.model_loader import load_uam_model


class TestP1Actuation(unittest.TestCase):
    def setUp(self) -> None:
        self.robot = load_uam_model()
        self.actuation = UamActuation(self.robot)

    def test_native_crocoddyl_mapping(self) -> None:
        native = self.actuation.crocoddyl_model()
        self.assertEqual(native.nu, self.actuation.nu)
        np.testing.assert_allclose(np.asarray(native.Wthrust), self.actuation.mapping)

    def test_hover_and_arm_reaction(self) -> None:
        q = self.robot.neutral_configuration(self.robot.config["initial_arm_configuration"])
        control = self.actuation.gravity_compensated_hover_control(q)
        baseline = generalized_acceleration(
            self.robot, self.actuation, q, np.zeros(self.robot.model.nv), control)
        stepped = control.copy()
        stepped[self.actuation.n_rotors] += 0.1
        response = generalized_acceleration(
            self.robot, self.actuation, q, np.zeros(self.robot.model.nv), stepped)
        self.assertLess(np.linalg.norm(baseline[:6]), 1e-8)
        self.assertGreater(np.linalg.norm(response[3:6] - baseline[3:6]), 1e-6)


if __name__ == "__main__":
    unittest.main()

