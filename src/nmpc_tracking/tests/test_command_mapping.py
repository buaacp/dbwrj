import os
import sys
import unittest

import numpy as np
import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from nmpc_tracking.frames import enu_to_ned_position, flu_to_frd_body_rate, ned_to_enu_position
from nmpc_tracking.arm_velocity_adapter import ArmVelocityAdapter
from nmpc_tracking.joint_mapping import JointMapping
from nmpc_tracking.px4_rate_thrust_adapter import ThrustMapper


class TestCommandMapping(unittest.TestCase):
    def test_enu_ned_roundtrip(self):
        v = np.array([1.0, 2.0, 3.0])
        np.testing.assert_allclose(ned_to_enu_position(enu_to_ned_position(v)), v)

    def test_flu_frd_body_rate(self):
        np.testing.assert_allclose(flu_to_frd_body_rate([1, 2, 3]), [1, -2, -3])

    def test_thrust_bounds(self):
        mapper = ThrustMapper(2.0, 9.81, 0.45)
        self.assertAlmostEqual(mapper.force_to_normalized(2.0 * 9.81), 0.45)
        self.assertEqual(mapper.force_to_normalized(-1.0), 0.0)
        self.assertEqual(mapper.force_to_normalized(1e9), 1.0)

    def test_planar_arm_split_mapping_excludes_shoulder_pan(self):
        cfg = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "config", "dual_mpc_pregrasp.yaml"))
        with open(cfg) as f:
            config = yaml.safe_load(f)
        names = config["arm"]["joint_names"]
        self.assertEqual(names, [
            "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_roll_joint"])
        self.assertNotIn("shoulder_pan_joint", names)
        mapping = JointMapping(names, config["arm"]["velocity_signs"], [1.0] * len(names))
        adapter = ArmVelocityAdapter(
            mapping,
            wrist_joint_name=config["arm"]["wrist_joint_name"],
            wrist_message_type=config["arm"]["wrist_command_message_type"],
        )
        arm_values, wrist_value = adapter.split_command_arrays([0.1, 0.2, 0.3, 0.4])
        np.testing.assert_allclose(arm_values, [0.1, 0.2, 0.3])
        self.assertAlmostEqual(wrist_value, 0.4)


if __name__ == "__main__":
    unittest.main()
