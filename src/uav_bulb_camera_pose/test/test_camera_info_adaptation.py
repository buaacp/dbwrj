#!/usr/bin/env python3
import os
import unittest

import cv2
import numpy as np

from uav_bulb_camera_pose.bundle_geometry import load_bundles
from uav_bulb_camera_pose.bundle_pose_core import AprilTagBundlePoseEstimator, Detection


class CameraInfoAdaptationTest(unittest.TestCase):
    def test_same_code_uses_different_camera_matrices(self):
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        bundles = load_bundles(os.path.join(root, 'config', 'bundles.yaml'))
        estimator = AprilTagBundlePoseEstimator(bundles, {'max_reprojection_rmse_px': 0.5})
        D = np.zeros((5, 1), dtype=np.float64)
        rvec = np.array([[0.05], [0.02], [0.01]], dtype=np.float64)
        tvec = np.array([[0.01], [-0.02], [0.80]], dtype=np.float64)
        for K in [
            np.array([[1397.0, 0.0, 960.0], [0.0, 1397.0, 540.0], [0.0, 0.0, 1.0]], dtype=np.float64),
            np.array([[931.0, 0.0, 640.0], [0.0, 931.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64),
            np.array([[466.0, 0.0, 320.0], [0.0, 466.0, 180.0], [0.0, 0.0, 1.0]], dtype=np.float64),
        ]:
            dets = []
            for tag_id in [200, 201, 202, 203]:
                pix, _ = cv2.projectPoints(bundles['bulb'].tags_by_id[tag_id].object_corners, rvec, tvec, K, D)
                dets.append(Detection(tag_id, pix.reshape(4, 2)))
            res = estimator.estimate_bundle(bundles['bulb'], dets, K, D)
            self.assertTrue(res.precision_valid)
            self.assertLess(np.linalg.norm(res.tvec.reshape(3) - tvec.reshape(3)), 1e-6)


if __name__ == '__main__':
    unittest.main()
