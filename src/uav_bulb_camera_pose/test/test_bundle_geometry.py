#!/usr/bin/env python3
import os
import unittest

import numpy as np

from uav_bulb_camera_pose.bundle_geometry import load_bundles


class BundleGeometryTest(unittest.TestCase):
    def test_loads_single_geometry_source(self):
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        bundles = load_bundles(os.path.join(root, 'config', 'bundles.yaml'))
        self.assertEqual(sorted(bundles.keys()), ['bulb', 'socket'])
        self.assertEqual(sorted(bundles['socket'].tags_by_id.keys()), [100, 101, 102, 103])
        self.assertEqual(sorted(bundles['bulb'].tags_by_id.keys()), [200, 201, 202, 203])
        for bundle in bundles.values():
            for tag in bundle.tags:
                self.assertEqual(tag.object_corners.shape, (4, 3))
                edges = np.linalg.norm(np.roll(tag.object_corners, -1, axis=0) - tag.object_corners, axis=1)
                self.assertLess(abs(edges[0] - tag.size_m), 1e-6)


if __name__ == '__main__':
    unittest.main()
