import os
import sys
import unittest

import numpy as np

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from nmpc_tracking.joint_mapping import JointMapping
from nmpc_tracking.arm_velocity_adapter import ArmVelocityAdapter


class TestJointMapping(unittest.TestCase):
    def test_name_order_mapping(self):
        mapping = JointMapping(["b", "a"], [1, -1], [1, 1])
        pos = mapping.positions_from_joint_state(["a", "b"], [10, 20])
        np.testing.assert_allclose(pos, [20, 10])

    def test_velocity_sign_and_limit(self):
        mapping = JointMapping(["j0", "j1"], [1, -1], [0.5, 0.2])
        np.testing.assert_allclose(mapping.command_to_message_order([1.0, 0.1]), [0.5, -0.1])

    def test_missing_joint_raises(self):
        mapping = JointMapping(["j0"], [1], [1])
        with self.assertRaises(KeyError):
            mapping.indices_for_joint_state(["other"])

    def test_split_wrist_command(self):
        mapping = JointMapping(["j0", "wrist"], [1, 1], [1, 1])
        adapter = ArmVelocityAdapter(mapping, wrist_joint_name="wrist")
        arm_values, wrist_value = adapter.split_command_arrays([0.1, 0.2])
        np.testing.assert_allclose(arm_values, [0.1])
        self.assertAlmostEqual(wrist_value, 0.2)


if __name__ == "__main__":
    unittest.main()
