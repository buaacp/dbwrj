#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os

import cv2
import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import CameraInfo, Image

from uav_bulb_camera_pose.bundle_geometry import load_bundles, matrix_to_pose, pose_to_matrix
from uav_bulb_camera_pose.bundle_pose_core import AprilTagBundlePoseEstimator
from uav_bulb_camera_pose.ros_image_numpy import bgr8_to_imgmsg, imgmsg_to_bgr8


def pose_msg_to_matrix(pose):
    p = pose.position
    q = pose.orientation
    return pose_to_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def T_link_optical():
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
    return T


def model_pose(states, name):
    if name not in states.name:
        return None
    return pose_msg_to_matrix(states.pose[states.name.index(name)])


def rmse_points(object_points, image_points, rvec, tvec, K, D):
    proj, _ = cv2.projectPoints(object_points, rvec, tvec, K, D)
    err = proj.reshape(-1, 2) - image_points
    return float(math.sqrt(np.mean(np.sum(err * err, axis=1))))


def pnp_debug(bundle, detections, K, D, ransac_px):
    object_points = []
    image_points = []
    tag_rows = []
    for det in detections:
        pts = bundle.object_points_for_detection(det.id)
        object_points.append(pts)
        image_points.append(det.corners)
        tag_rows.append({
            'tag_id': det.id,
            'image_corners_px': det.corners.tolist(),
            'object_points_m': pts.tolist(),
            'decision_margin': det.decision_margin,
            'hamming': det.hamming,
        })
    if not object_points:
        return {'tags': tag_rows, 'ok': False, 'message': 'no visible tags'}
    object_points = np.vstack(object_points).astype(np.float64)
    image_points = np.vstack(image_points).astype(np.float64)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points, image_points, K, D,
        iterationsCount=100, reprojectionError=ransac_px, confidence=0.99,
        flags=cv2.SOLVEPNP_ITERATIVE)
    out = {'tags': tag_rows, 'ok': bool(ok), 'ransac_reprojection_error_px': ransac_px}
    if not ok:
        out['message'] = 'solvePnPRansac failed'
        return out
    idx = inliers.reshape(-1) if inliers is not None else np.arange(len(object_points))
    out.update({
        'inlier_count': int(len(idx)),
        'rvec_before_refine': np.asarray(rvec).reshape(3).tolist(),
        'tvec_before_refine': np.asarray(tvec).reshape(3).tolist(),
        'rmse_before_refine_all_px': rmse_points(object_points, image_points, rvec, tvec, K, D),
        'rmse_before_refine_inliers_px': rmse_points(object_points[idx], image_points[idx], rvec, tvec, K, D),
    })
    if len(idx) >= 4 and hasattr(cv2, 'solvePnPRefineLM'):
        rvec2, tvec2 = cv2.solvePnPRefineLM(object_points[idx], image_points[idx], K, D, rvec, tvec)
    else:
        rvec2, tvec2 = rvec, tvec
    out.update({
        'rvec_after_refine': np.asarray(rvec2).reshape(3).tolist(),
        'tvec_after_refine': np.asarray(tvec2).reshape(3).tolist(),
        'rmse_after_refine_all_px': rmse_points(object_points, image_points, rvec2, tvec2, K, D),
        'rmse_after_refine_inliers_px': rmse_points(object_points[idx], image_points[idx], rvec2, tvec2, K, D),
    })
    return out


def single_tag_ippe(bundle, det, K, D):
    tag = bundle.tags_by_id[det.id]
    s = tag.size_m / 2.0
    obj = np.array([[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]], dtype=np.float64)
    img = np.array([det.corners[3], det.corners[2], det.corners[1], det.corners[0]], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return {'tag_id': det.id, 'ok': bool(ok), 'rvec': np.asarray(rvec).reshape(3).tolist() if ok else None, 'tvec': np.asarray(tvec).reshape(3).tolist() if ok else None}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bundle-yaml', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--image-topic', default='/camera/camera/color/image_raw')
    parser.add_argument('--camera-info-topic', default='/camera/camera/color/camera_info')
    parser.add_argument('--ransac-px', type=float, default=6.0)
    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    rospy.init_node('capture_pnp_debug_frame', anonymous=True)

    image_msg = rospy.wait_for_message(args.image_topic, Image, timeout=10.0)
    info_msg = rospy.wait_for_message(args.camera_info_topic, CameraInfo, timeout=10.0)
    states = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=10.0)
    image = imgmsg_to_bgr8(image_msg)
    cv2.imwrite(os.path.join(args.output_dir, 'pnp_debug_raw_image.png'), image)
    K = np.array(info_msg.K, dtype=np.float64).reshape(3, 3)
    D = np.array(info_msg.D, dtype=np.float64).reshape(-1, 1) if info_msg.D else np.zeros((0, 1), dtype=np.float64)

    bundles = load_bundles(args.bundle_yaml)
    estimator = AprilTagBundlePoseEstimator(bundles, {'ransac_reprojection_error_px': args.ransac_px})
    detections = estimator.detect(image)
    results = estimator.estimate_all(detections, K, D)
    debug = cv2.imread(os.path.join(args.output_dir, 'pnp_debug_raw_image.png'))
    from uav_bulb_camera_pose.bundle_pose_core import draw_debug
    cv2.imwrite(os.path.join(args.output_dir, 'pnp_debug_image.png'), draw_debug(image, detections, results))

    T_W_C = model_pose(states, 'd435i_color_camera_rig').dot(T_link_optical())
    truth = {}
    for name, model in [('socket', 'socket_tag_ring'), ('bulb', 'bulb_tag_ring')]:
        T_W_O = model_pose(states, model)
        if T_W_O is not None:
            T_C_O = np.linalg.inv(T_W_C).dot(T_W_O)
            p, q = matrix_to_pose(T_C_O)
            truth[name] = {'tvec_gt': p.tolist(), 'quat_gt_xyzw': q.tolist()}

    report = {'camera_info': {'K': K.tolist(), 'D': D.reshape(-1).tolist(), 'stamp': info_msg.header.stamp.to_sec()},
              'image_stamp': image_msg.header.stamp.to_sec(),
              'detections': [],
              'objects': {},
              'gazebo_truth_camera_frame': truth}
    for det in detections:
        report['detections'].append({'tag_id': det.id, 'corners_px': det.corners.tolist(), 'decision_margin': det.decision_margin, 'hamming': det.hamming})
    for name, bundle in bundles.items():
        obj_dets = [d for d in detections if d.id in bundle.tags_by_id]
        report['objects'][name] = pnp_debug(bundle, obj_dets, K, D, args.ransac_px)
        report['objects'][name]['single_tag_ippe'] = [single_tag_ippe(bundle, d, K, D) for d in obj_dets]

    with open(os.path.join(args.output_dir, 'pnp_debug.json'), 'w') as f:
        json.dump(report, f, indent=2, sort_keys=True)
    print(os.path.join(args.output_dir, 'pnp_debug.json'))


if __name__ == '__main__':
    main()
