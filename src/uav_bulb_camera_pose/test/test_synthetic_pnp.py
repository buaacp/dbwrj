#!/usr/bin/env python3
import os
import unittest

import cv2
import numpy as np

from uav_bulb_camera_pose.bundle_geometry import load_bundles
from uav_bulb_camera_pose.bundle_pose_core import AprilTagBundlePoseEstimator, Detection


class SyntheticPnpTest(unittest.TestCase):
    def setUp(self):
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        self.bundles = load_bundles(os.path.join(root, 'config', 'bundles.yaml'))
        self.estimator = AprilTagBundlePoseEstimator(self.bundles, {
            'min_precision_visible_tags': 2,
            'min_inlier_corners': 8,
            'max_reprojection_rmse_px': 0.5,
            'min_positive_depth_m': 0.10,
            'max_positive_depth_m': 5.0,
        })
        self.K = np.array([[1397.0, 0.0, 960.0], [0.0, 1397.0, 540.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        self.D = np.zeros((5, 1), dtype=np.float64)

    def make_detections(self, bundle, ids):
        rvec = np.array([[0.08], [-0.05], [0.03]], dtype=np.float64)
        tvec = np.array([[0.02], [0.01], [0.70]], dtype=np.float64)
        dets = []
        for tag_id in ids:
            pix, _ = cv2.projectPoints(bundle.tags_by_id[tag_id].object_corners, rvec, tvec, self.K, self.D)
            dets.append(Detection(tag_id, pix.reshape(4, 2)))
        return dets, rvec, tvec

    def test_multi_tag_joint_pnp_precision_valid(self):
        dets, _rvec, tvec = self.make_detections(self.bundles['socket'], [100, 101, 102])
        res = self.estimator.estimate_bundle(self.bundles['socket'], dets, self.K, self.D)
        self.assertTrue(res.precision_valid)
        self.assertEqual(res.visible_tag_count, 3)
        self.assertGreaterEqual(res.inlier_corner_count, 8)
        self.assertLess(np.linalg.norm(res.tvec.reshape(3) - tvec.reshape(3)), 1e-6)

    def test_single_tag_is_degraded_not_precision(self):
        dets, _rvec, _tvec = self.make_detections(self.bundles['socket'], [100])
        res = self.estimator.estimate_bundle(self.bundles['socket'], dets, self.K, self.D)
        self.assertTrue(res.valid_pose)
        self.assertTrue(res.degraded_pose_valid)
        self.assertFalse(res.precision_valid)


if __name__ == '__main__':
    unittest.main()
