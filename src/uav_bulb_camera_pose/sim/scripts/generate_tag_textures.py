#!/usr/bin/env python3
import argparse
import os

import cv2
import numpy as np

from uav_bulb_camera_pose.bundle_geometry import load_bundles


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bundle-yaml', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--pixels', type=int, default=600)
    parser.add_argument('--marker-fraction', type=float, default=0.75)
    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    bundles = load_bundles(args.bundle_yaml)
    for bundle in bundles.values():
        for tag in bundle.tags:
            marker_pixels = int(round(args.pixels * args.marker_fraction))
            marker = cv2.aruco.generateImageMarker(dictionary, tag.id, marker_pixels)
            canvas = np.ones((args.pixels, args.pixels), dtype=np.uint8) * 255
            offset = (args.pixels - marker_pixels) // 2
            canvas[offset:offset + marker_pixels, offset:offset + marker_pixels] = marker
            path = os.path.join(args.output_dir, 'tag36h11_%03d.png' % tag.id)
            cv2.imwrite(path, cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR))
            print(path)


if __name__ == '__main__':
    main()
