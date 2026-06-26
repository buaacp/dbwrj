import os
import sys
import unittest

import numpy as np
import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from nmpc_tracking.trajectory_reference import TrajectoryReference


class TestReferenceInterpolation(unittest.TestCase):
    def setUp(self):
        cfg = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "config", "dual_mpc_pregrasp.yaml"))
        with open(cfg) as f:
            config = yaml.safe_load(f)
        self.path = config["trajectory"]["offline_npz"]
        self.ref = TrajectoryReference.from_npz(self.path, hold_after_s=5.0)

    def test_original_nodes_reproduce(self):
        for idx in [0, 10, 40]:
            sample = self.ref.sample(idx * self.ref.snapshot.dt)
            state = self.ref.snapshot.states[idx]
            np.testing.assert_allclose(sample.position_w, state[:3], atol=1e-12)
            np.testing.assert_allclose(sample.quaternion_xyzw, state[3:7], atol=1e-12)

    def test_resample_to_50hz(self):
        samples = self.ref.resample(0.02, until_s=0.30)
        self.assertEqual(len(samples), 16)
        self.assertEqual(samples[0].joint_position.shape[0], self.ref.snapshot.arm_joint_count)

    def test_terminal_hold_zero_rates(self):
        sample = self.ref.sample(self.ref.snapshot.duration + 1.0)
        np.testing.assert_allclose(sample.velocity_w, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(sample.body_rate, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(sample.joint_velocity, np.zeros_like(sample.joint_velocity), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
