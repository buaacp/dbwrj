#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sys

import cv2
import numpy as np

from uav_bulb_camera_pose.bundle_geometry import load_bundles, pose_to_matrix
from uav_bulb_camera_pose.bundle_pose_core import AprilTagBundlePoseEstimator, Detection


def default_K(width, height, hfov_deg):
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    fy = fx
    return np.array([[fx, 0.0, width / 2.0], [0.0, fy, height / 2.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def project_bundle(bundle, rvec, tvec, K, D, visible_ids, pixel_noise=0.0):
    detections = []
    for tag_id in visible_ids:
        pts = bundle.tags_by_id[tag_id].object_corners
        pix, _ = cv2.projectPoints(pts, rvec, tvec, K, D)
        corners = pix.reshape(4, 2)
        if pixel_noise > 0.0:
            corners = corners + np.random.normal(0.0, pixel_noise, corners.shape)
        detections.append(Detection(tag_id, corners))
    return detections


def pose_error(result, rvec_gt, tvec_gt):
    R_gt, _ = cv2.Rodrigues(rvec_gt)
    R_est = result.rotation_matrix
    pos = float(np.linalg.norm(result.tvec.reshape(3) - tvec_gt.reshape(3)))
    trace = (np.trace(R_gt.T.dot(R_est)) - 1.0) / 2.0
    rot = math.degrees(math.acos(max(min(float(trace), 1.0), -1.0)))
    axis_gt = R_gt.dot(np.array([0.0, 0.0, 1.0]))
    axis_est = R_est.dot(np.array([0.0, 0.0, 1.0]))
    dot = float(np.dot(axis_gt, axis_est) / max(np.linalg.norm(axis_gt) * np.linalg.norm(axis_est), 1e-9))
    axis = math.degrees(math.acos(max(min(dot, 1.0), -1.0)))
    return pos, rot, axis


def summarize(rows, out_path):
    lines = []
    for obj in sorted(set(r['object'] for r in rows)):
        subset = [r for r in rows if r['object'] == obj]
        pos = np.array([float(r['position_error_m']) for r in subset], dtype=np.float64)
        axis = np.array([float(r['axis_error_deg']) for r in subset], dtype=np.float64)
        valid = np.array([int(r['precision_valid']) for r in subset], dtype=np.float64)
        false_valid = np.array([int(r['false_valid']) for r in subset], dtype=np.float64)
        lines.append(obj)
        lines.append('position_error_m median=%.6f mean=%.6f p95=%.6f' % (np.median(pos), np.mean(pos), np.percentile(pos, 95)))
        lines.append('axis_error_deg median=%.6f mean=%.6f p95=%.6f' % (np.median(axis), np.mean(axis), np.percentile(axis, 95)))
        lines.append('valid_ratio=%.6f' % float(np.mean(valid)))
        lines.append('false_valid_ratio=%.6f' % float(np.mean(false_valid)))
        lines.append('max_outage_duration_s=%.3f' % max_outage(subset, 30.0))
        lines.append('')
    with open(out_path, 'w') as f:
        f.write('\n'.join(lines))


def max_outage(rows, hz):
    max_run = 0
    run = 0
    for r in rows:
        if int(r['precision_valid']):
            max_run = max(max_run, run)
            run = 0
        else:
            run += 1
    max_run = max(max_run, run)
    return max_run / hz


def run_scenario(name, bundles, estimator, result_dir, visible_by_obj, pixel_noise=0.0, resolution=(1920, 1080), hfov=69.0):
    width, height = resolution
    K = default_K(width, height, hfov)
    D = np.zeros((5, 1), dtype=np.float64)
    rows = []
    csv_path = os.path.join(result_dir, name + '.csv')
    np.random.seed(7)
    for frame in range(180):
        stamp = frame / 30.0
        yaw = 0.03 * math.sin(frame / 40.0)
        pitch = 0.02 * math.cos(frame / 50.0)
        rvec_socket = np.array([[pitch], [yaw], [0.02]], dtype=np.float64)
        tvec_socket = np.array([[0.02 * math.sin(frame / 30.0)], [0.01], [0.65]], dtype=np.float64)
        rvec_bulb = np.array([[pitch], [yaw], [0.02 + 0.01 * math.sin(frame / 20.0)]], dtype=np.float64)
        tvec_bulb = tvec_socket + np.array([[0.0], [0.0], [0.10]], dtype=np.float64)
        all_dets = []
        gt = {
            'socket': (rvec_socket, tvec_socket),
            'bulb': (rvec_bulb, tvec_bulb),
        }
        for obj in ['socket', 'bulb']:
            visible = visible_by_obj.get(obj, [t.id for t in bundles[obj].tags])
            all_dets.extend(project_bundle(bundles[obj], gt[obj][0], gt[obj][1], K, D, visible, pixel_noise))
        results = estimator.estimate_all(all_dets, K, D)
        for obj, result in results.items():
            if result.valid_pose:
                pos, rot, axis = pose_error(result, gt[obj][0], gt[obj][1])
            else:
                pos, rot, axis = float('nan'), float('nan'), float('nan')
            should_be_valid = len(visible_by_obj.get(obj, [t.id for t in bundles[obj].tags])) >= 2
            rows.append({
                'stamp': '%.6f' % stamp,
                'object': obj,
                'position_error_m': '%.9f' % pos if np.isfinite(pos) else 'nan',
                'rotation_error_deg': '%.6f' % rot if np.isfinite(rot) else 'nan',
                'axis_error_deg': '%.6f' % axis if np.isfinite(axis) else 'nan',
                'reprojection_rmse_px': '%.6f' % result.reprojection_rmse_px if np.isfinite(result.reprojection_rmse_px) else 'inf',
                'visible_tag_count': result.visible_tag_count,
                'inlier_corner_count': result.inlier_corner_count,
                'precision_valid': int(result.precision_valid),
                'degraded_pose_valid': int(result.degraded_pose_valid),
                'false_valid': int(result.precision_valid and not should_be_valid),
            })
    with open(csv_path, 'w') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    summarize(rows, os.path.join(result_dir, name + '_summary.txt'))
    return csv_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bundle-yaml', required=True)
    parser.add_argument('--result-dir', required=True)
    args = parser.parse_args()
    os.makedirs(args.result_dir, exist_ok=True)
    bundles = load_bundles(args.bundle_yaml)
    estimator = AprilTagBundlePoseEstimator(bundles, {
        'min_precision_visible_tags': 2,
        'min_inlier_corners': 8,
        'max_reprojection_rmse_px': 2.5,
        'min_positive_depth_m': 0.10,
        'max_positive_depth_m': 5.0,
    })
    scenarios = {
        'T1_static_unoccluded': ({}, 0.05, (1920, 1080)),
        'T2_view_scan': ({}, 0.08, (1920, 1080)),
        'T3_socket_one_tag_occluded': ({'socket': [100, 101, 102]}, 0.08, (1920, 1080)),
        'T4_socket_two_tags_occluded': ({'socket': [100, 101]}, 0.08, (1920, 1080)),
        'T5_socket_single_tag': ({'socket': [100]}, 0.05, (1920, 1080)),
        'T6_socket_all_occluded': ({'socket': []}, 0.05, (1920, 1080)),
        'T9_resolution_1280x720': ({}, 0.08, (1280, 720)),
        'T10_noise_blur_proxy': ({}, 0.45, (1280, 720)),
    }
    paths = []
    for name, (visible, noise, resolution) in scenarios.items():
        paths.append(run_scenario(name, bundles, estimator, args.result_dir, visible, noise, resolution))
    print('\n'.join(paths))


if __name__ == '__main__':
    main()
