import os
import sys
import unittest

import numpy as np
import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from nmpc_tracking.acados_model import controller_dimensions, hover_command
from nmpc_tracking.robot_layout import (
    LEGACY_6DOF_MESSAGE,
    assert_layout_consistency,
    load_robot_layout,
)
from nmpc_tracking.trajectory_reference import TrajectoryReference


class TestRobotLayoutConsistency(unittest.TestCase):
    def setUp(self):
        cfg = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "config", "dual_mpc_pregrasp.yaml"))
        with open(cfg) as f:
            self.config = yaml.safe_load(f)
        self.layout = load_robot_layout(self.config)

    def test_layout_matches_trajectory_and_controller(self):
        dims = controller_dimensions(self.config, self.layout)
        self.assertEqual(self.layout.arm_dof, self.layout.trajectory_arm_dof)
        self.assertEqual(dims["joint_count"], self.layout.arm_dof)
        self.assertEqual(dims["command_dim"], 4 + self.layout.arm_dof)
        self.assertEqual(dims["state_dim"], 16 + 3 * self.layout.arm_dof)
        self.assertEqual(self.layout.arm_dof, 4)
        self.assertEqual(self.layout.arm_joint_names, [
            "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_roll_joint"])
        self.assertNotIn("shoulder_pan_joint", self.layout.arm_joint_names)
        self.assertNotIn("shoulder_pan_joint", self.layout.trajectory_q_names)
        self.assertNotIn("shoulder_pan_joint_torque_Nm", self.layout.trajectory_control_names)
        assert_layout_consistency(
            self.layout, dims["state_dim"], dims["command_dim"], dims["control_rate_dim"])

    def test_mass_drives_hover_thrust(self):
        hover = hover_command(self.config, self.layout)[0]
        expected = self.layout.total_mass_kg * float(self.config["vehicle"]["gravity_mps2"])
        self.assertLess(abs(hover - expected), 1e-9)

    def test_arm_names_and_indices(self):
        self.assertEqual(self.layout.arm_joint_names, self.config["arm"]["joint_names"])
        self.assertNotIn("shoulder_pan_joint", self.config["arm"]["joint_names"])
        self.assertEqual(len(self.layout.q_indices), self.layout.arm_dof)
        self.assertEqual(len(self.layout.v_indices), self.layout.arm_dof)
        self.assertTrue(np.all(np.isfinite(self.layout.velocity_limits)))

    def test_legacy_trajectory_rejected(self):
        old = "/home/zlhq/px4_fly_ws/uam_ocp/results/p2_bulb_pregrasp/runs/rerun_20260623_001/whole_body/trajectory.npz"
        if not os.path.exists(old):
            self.skipTest("legacy trajectory fixture absent")
        with self.assertRaisesRegex(ValueError, "Legacy 6-DoF trajectory"):
            TrajectoryReference.from_npz(old)


if __name__ == "__main__":
    unittest.main()
