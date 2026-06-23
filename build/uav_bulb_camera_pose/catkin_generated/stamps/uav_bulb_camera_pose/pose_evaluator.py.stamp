#!/usr/bin/env python3
import csv
import math
import os

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, Int32

from uav_bulb_camera_pose.bundle_geometry import pose_to_matrix, matrix_to_pose, quat_to_rot


def pose_msg_to_matrix(pose):
    p = pose.position
    q = pose.orientation
    return pose_to_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def rotation_error_deg(R_gt, R_est):
    v = (np.trace(R_gt.T.dot(R_est)) - 1.0) / 2.0
    return math.degrees(math.acos(max(min(float(v), 1.0), -1.0)))


class MetricLatch(object):
    def __init__(self, prefix):
        self.pose = None
        self.valid = False
        self.rmse = -1.0
        self.tags = 0
        self.inliers = 0
        self.last_written_stamp = None
        rospy.Subscriber(prefix + '/pose_camera', PoseStamped, self.pose_cb, queue_size=20)
        rospy.Subscriber(prefix + '/precision_valid', Bool, lambda m: setattr(self, 'valid', bool(m.data)), queue_size=20)
        rospy.Subscriber(prefix + '/reprojection_error_px', Float32, lambda m: setattr(self, 'rmse', float(m.data)), queue_size=20)
        rospy.Subscriber(prefix + '/visible_tag_count', Int32, lambda m: setattr(self, 'tags', int(m.data)), queue_size=20)
        rospy.Subscriber(prefix + '/inlier_corner_count', Int32, lambda m: setattr(self, 'inliers', int(m.data)), queue_size=20)

    def pose_cb(self, msg):
        self.pose = msg


class PoseEvaluator(object):
    def __init__(self):
        self.camera_model = rospy.get_param('~camera_model_name', 'd435i_color_camera_rig')
        self.socket_model = rospy.get_param('~socket_model_name', 'socket_tag_ring')
        self.bulb_model = rospy.get_param('~bulb_model_name', 'bulb_tag_ring')
        self.gazebo_camera_link_to_optical = bool(rospy.get_param('~gazebo_camera_link_to_optical', True))
        self.scenario = rospy.get_param('~scenario_name', 'manual')
        result_dir = rospy.get_param('~result_dir', os.path.join(os.getcwd(), 'src/uav_bulb_camera_pose/results'))
        os.makedirs(result_dir, exist_ok=True)
        self.csv_path = os.path.join(result_dir, self.scenario + '.csv')
        self.rows = []
        self.closed = False
        self.models = None
        self.latches = {'socket': MetricLatch('/socket'), 'bulb': MetricLatch('/bulb')}
        self.csv_file = open(self.csv_path, 'w')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=[
            'stamp', 'object', 'position_error_m', 'rotation_error_deg', 'axis_error_deg',
            'reprojection_rmse_px', 'visible_tag_count', 'inlier_corner_count', 'precision_valid',
            'false_valid',
        ])
        self.writer.writeheader()
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_cb, queue_size=5)
        rospy.Timer(rospy.Duration(0.1), self.tick)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('pose_evaluator writing %s', self.csv_path)

    def model_cb(self, msg):
        self.models = msg

    def model_pose(self, name):
        if self.models is None or name not in self.models.name:
            return None
        return pose_msg_to_matrix(self.models.pose[self.models.name.index(name)])

    def tick(self, _event):
        if self.closed:
            return
        T_W_C_link = self.model_pose(self.camera_model)
        if T_W_C_link is None:
            return
        T_W_C = T_W_C_link.dot(self.T_link_optical())
        for obj, model_name in [('socket', self.socket_model), ('bulb', self.bulb_model)]:
            latch = self.latches[obj]
            if latch.pose is None:
                continue
            stamp = latch.pose.header.stamp.to_sec()
            if latch.last_written_stamp == stamp:
                continue
            if latch.rmse < 0.0 or latch.tags < 1 or latch.inliers < 1:
                continue
            T_W_O = self.model_pose(model_name)
            if T_W_O is None:
                continue
            T_C_O_gt = np.linalg.inv(T_W_C).dot(T_W_O)
            T_C_O_est = pose_msg_to_matrix(latch.pose.pose)
            p_gt, _ = matrix_to_pose(T_C_O_gt)
            p_est, _ = matrix_to_pose(T_C_O_est)
            R_gt = T_C_O_gt[:3, :3]
            R_est = T_C_O_est[:3, :3]
            axis_gt = R_gt.dot(np.array([0.0, 0.0, 1.0]))
            axis_est = R_est.dot(np.array([0.0, 0.0, 1.0]))
            axis_dot = float(np.dot(axis_gt, axis_est) / max(np.linalg.norm(axis_gt) * np.linalg.norm(axis_est), 1e-9))
            row = {
                'stamp': '%.9f' % latch.pose.header.stamp.to_sec(),
                'object': obj,
                'position_error_m': '%.9f' % float(np.linalg.norm(p_est - p_gt)),
                'rotation_error_deg': '%.6f' % rotation_error_deg(R_gt, R_est),
                'axis_error_deg': '%.6f' % math.degrees(math.acos(max(min(axis_dot, 1.0), -1.0))),
                'reprojection_rmse_px': '%.6f' % latch.rmse,
                'visible_tag_count': latch.tags,
                'inlier_corner_count': latch.inliers,
                'precision_valid': int(latch.valid),
                'false_valid': 0,
            }
            self.rows.append(row)
            self.writer.writerow(row)
            self.csv_file.flush()
            latch.last_written_stamp = stamp

    def T_link_optical(self):
        T = np.eye(4, dtype=np.float64)
        if self.gazebo_camera_link_to_optical:
            # Gazebo camera renders along link +X. OpenCV optical coordinates use
            # +Z forward, +X right, +Y down.
            T[:3, :3] = np.array([
                [0.0, 0.0, 1.0],
                [-1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
            ], dtype=np.float64)
        return T

    def shutdown(self):
        self.closed = True
        self.csv_file.close()
        if not self.rows:
            return
        summary_path = self.csv_path.replace('.csv', '_summary.txt')
        with open(summary_path, 'w') as f:
            for obj in ['socket', 'bulb']:
                vals = [float(r['position_error_m']) for r in self.rows if r['object'] == obj]
                axes = [float(r['axis_error_deg']) for r in self.rows if r['object'] == obj]
                valids = [int(r['precision_valid']) for r in self.rows if r['object'] == obj]
                if not vals:
                    continue
                f.write('%s\n' % obj)
                f.write('position_error_m median=%.6f mean=%.6f p95=%.6f\n' % (np.median(vals), np.mean(vals), np.percentile(vals, 95)))
                f.write('axis_error_deg median=%.6f mean=%.6f p95=%.6f\n' % (np.median(axes), np.mean(axes), np.percentile(axes, 95)))
                f.write('valid_ratio=%.6f\n\n' % (sum(valids) / float(len(valids))))


if __name__ == '__main__':
    rospy.init_node('pose_evaluator')
    PoseEvaluator()
    rospy.spin()
