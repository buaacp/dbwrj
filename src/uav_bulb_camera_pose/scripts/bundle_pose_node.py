#!/usr/bin/env python3
import os
import sys
from collections import deque

import cv2
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, Int32

from uav_bulb_camera_pose.bundle_geometry import slerp
from uav_bulb_camera_pose.bundle_geometry import load_bundles
from uav_bulb_camera_pose.bundle_pose_core import AprilTagBundlePoseEstimator, draw_debug
from uav_bulb_camera_pose.ros_image_numpy import bgr8_to_imgmsg, imgmsg_to_bgr8


def resolve_ros_path(path):
    if not path:
        return path
    if path.startswith('$(find '):
        pkg = path.split('$(find ', 1)[1].split(')', 1)[0]
        rest = path.split(')', 1)[1].lstrip('/')
        try:
            import rospkg
            return os.path.join(rospkg.RosPack().get_path(pkg), rest)
        except Exception:
            return path
    return os.path.expanduser(path)


class BundlePoseNode(object):
    def __init__(self):
        self.latest_info = None
        self.latest_info_stamp = None
        self.info_buffer = deque(maxlen=120)
        self.filter_state = {}

        bundle_yaml = rospy.get_param('~bundle_yaml', rospy.get_param('~socket_bundle_yaml', ''))
        self.bundle_yaml = resolve_ros_path(bundle_yaml)
        self.bundles = load_bundles(self.bundle_yaml)

        self.params = {
            'min_precision_visible_tags': rospy.get_param('~min_precision_visible_tags', 2),
            'min_inlier_corners': rospy.get_param('~min_inlier_corners', 8),
            'max_reprojection_rmse_px': rospy.get_param('~max_reprojection_rmse_px', 2.5),
            'min_positive_depth_m': rospy.get_param('~min_positive_depth_m', 0.10),
            'max_positive_depth_m': rospy.get_param('~max_positive_depth_m', 5.0),
            'ransac_reprojection_error_px': rospy.get_param('~ransac_reprojection_error_px', 4.0),
            'ransac_iterations': rospy.get_param('~ransac_iterations', 100),
        }
        self.max_dt = float(rospy.get_param('~max_image_camera_info_dt_sec', 0.050))
        self.frame_override = rospy.get_param('~camera_frame_override', '')
        self.enable_filter = bool(rospy.get_param('~enable_filter', True))
        self.filter_alpha = float(rospy.get_param('~filter_alpha', 0.55))
        self.filter_max_jump_m = float(rospy.get_param('~filter_max_jump_m', 0.15))
        self.publish_debug = bool(rospy.get_param('~publish_debug_image', True))

        self.estimator = AprilTagBundlePoseEstimator(self.bundles, self.params)

        self.publishers = {}
        for name in ['socket', 'bulb']:
            base = '/' + name
            self.publishers[name] = {
                'pose': rospy.Publisher(base + '/pose_camera', PoseStamped, queue_size=5),
                'axis': rospy.Publisher(base + '/axis_camera', Vector3Stamped, queue_size=5),
                'precision': rospy.Publisher(base + '/precision_valid', Bool, queue_size=5),
                'degraded': rospy.Publisher(base + '/degraded_pose_valid', Bool, queue_size=5),
                'rmse': rospy.Publisher(base + '/reprojection_error_px', Float32, queue_size=5),
                'tags': rospy.Publisher(base + '/visible_tag_count', Int32, queue_size=5),
                'inliers': rospy.Publisher(base + '/inlier_corner_count', Int32, queue_size=5),
            }
        self.debug_pub = rospy.Publisher('/bulb_vision/debug_image', Image, queue_size=2)
        self.diag_pub = rospy.Publisher('/bulb_vision/diagnostics', DiagnosticArray, queue_size=5)

        image_topic = rospy.get_param('~image_topic', '/camera/camera/color/image_raw')
        info_topic = rospy.get_param('~camera_info_topic', '/camera/camera/color/camera_info')
        self.info_sub = rospy.Subscriber(info_topic, CameraInfo, self.info_cb, queue_size=3)
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_cb, queue_size=1, buff_size=2 ** 24)
        rospy.loginfo('bundle_pose_node using %s and %s, bundle geometry %s', image_topic, info_topic, self.bundle_yaml)

    def info_cb(self, msg):
        self.latest_info = msg
        self.latest_info_stamp = msg.header.stamp
        self.info_buffer.append(msg)

    @staticmethod
    def camera_matrix_and_distortion(info):
        K = np.array(info.K, dtype=np.float64).reshape(3, 3)
        D = np.array(info.D, dtype=np.float64).reshape(-1, 1) if info.D else np.zeros((0, 1), dtype=np.float64)
        return K, D

    def camera_info_for_stamp(self, stamp):
        if not self.info_buffer:
            return self.latest_info
        return min(self.info_buffer, key=lambda info: abs((stamp - info.header.stamp).to_sec()))

    def image_cb(self, msg):
        info = self.camera_info_for_stamp(msg.header.stamp)
        if info is None:
            self.publish_diagnostics(msg.header.stamp, False, 'waiting_for_camera_info', [])
            return

        dt = abs((msg.header.stamp - info.header.stamp).to_sec())
        time_ok = dt <= self.max_dt
        frame_id = self.frame_override or info.header.frame_id or msg.header.frame_id
        if not frame_id:
            frame_id = 'camera_color_optical_frame'
        K, D = self.camera_matrix_and_distortion(info)

        try:
            image = imgmsg_to_bgr8(msg)
        except Exception as exc:
            self.publish_diagnostics(msg.header.stamp, False, 'image conversion failed: %s' % exc, [])
            return

        detections = self.estimator.detect(image)
        results = self.estimator.estimate_all(detections, K, D)
        for res in results.values():
            if not time_ok:
                res.precision_valid = False
                res.message = 'image-camera_info dt %.3fs exceeds %.3fs' % (dt, self.max_dt)

        for name in ['socket', 'bulb']:
            if name in results:
                self.publish_result(name, results[name], msg.header.stamp, frame_id)

        self.publish_diagnostics(msg.header.stamp, time_ok, 'ok' if time_ok else 'stale_camera_info', results.values(), dt)
        if self.publish_debug and self.debug_pub.get_num_connections() > 0:
            dbg = draw_debug(image, detections, results)
            out_msg = bgr8_to_imgmsg(dbg)
            out_msg.header.stamp = msg.header.stamp
            out_msg.header.frame_id = frame_id
            self.debug_pub.publish(out_msg)

    def filtered_pose(self, name, p, q, allow_filter):
        if not self.enable_filter or not allow_filter:
            self.filter_state[name] = (p.copy(), q.copy())
            return p, q
        if name not in self.filter_state:
            self.filter_state[name] = (p.copy(), q.copy())
            return p, q
        prev_p, prev_q = self.filter_state[name]
        if np.linalg.norm(p - prev_p) > self.filter_max_jump_m:
            self.filter_state[name] = (p.copy(), q.copy())
            return p, q
        out_p = self.filter_alpha * p + (1.0 - self.filter_alpha) * prev_p
        out_q = slerp(prev_q, q, self.filter_alpha)
        self.filter_state[name] = (out_p.copy(), out_q.copy())
        return out_p, out_q

    def publish_result(self, name, res, stamp, frame_id):
        pubs = self.publishers[name]
        pubs['precision'].publish(Bool(bool(res.precision_valid)))
        pubs['degraded'].publish(Bool(bool(res.degraded_pose_valid)))
        pubs['rmse'].publish(Float32(float(res.reprojection_rmse_px if np.isfinite(res.reprojection_rmse_px) else -1.0)))
        pubs['tags'].publish(Int32(int(res.visible_tag_count)))
        pubs['inliers'].publish(Int32(int(res.inlier_corner_count)))

        if not res.valid_pose:
            return
        p, q = self.filtered_pose(name, res.tvec.astype(np.float64), res.quaternion_xyzw.astype(np.float64), res.precision_valid)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = [float(v) for v in p]
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = [float(v) for v in q]
        pubs['pose'].publish(pose)

        axis = Vector3Stamped()
        axis.header.stamp = stamp
        axis.header.frame_id = frame_id
        axis.vector.x, axis.vector.y, axis.vector.z = [float(v) for v in res.axis]
        pubs['axis'].publish(axis)

    def publish_diagnostics(self, stamp, ok, message, results, dt=None):
        arr = DiagnosticArray()
        arr.header.stamp = stamp
        status = DiagnosticStatus()
        status.name = 'bulb_vision/bundle_pose_node'
        status.hardware_id = 'd435i_rgb_apriltag'
        status.level = DiagnosticStatus.OK if ok else DiagnosticStatus.WARN
        status.message = message
        values = []
        if dt is not None:
            values.append(KeyValue('image_camera_info_dt_sec', '%.6f' % dt))
        if self.latest_info is not None:
            values.extend([
                KeyValue('camera_info_width', str(self.latest_info.width)),
                KeyValue('camera_info_height', str(self.latest_info.height)),
                KeyValue('camera_info_frame_id', self.latest_info.header.frame_id),
                KeyValue('distortion_model', self.latest_info.distortion_model),
            ])
        for res in results:
            values.extend([
                KeyValue(res.bundle_name + '_visible_tag_count', str(res.visible_tag_count)),
                KeyValue(res.bundle_name + '_precision_valid', str(res.precision_valid)),
                KeyValue(res.bundle_name + '_rmse_px', '%.3f' % res.reprojection_rmse_px if np.isfinite(res.reprojection_rmse_px) else 'inf'),
                KeyValue(res.bundle_name + '_message', res.message),
            ])
        status.values = values
        arr.status = [status]
        self.diag_pub.publish(arr)


if __name__ == '__main__':
    rospy.init_node('bundle_pose_node')
    try:
        BundlePoseNode()
        rospy.spin()
    except Exception as exc:
        rospy.logerr('bundle_pose_node fatal: %s', exc)
        raise
