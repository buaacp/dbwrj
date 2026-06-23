#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import csv
import math
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64MultiArray, Int32


def parse_vector(value, default):
    if isinstance(value, str):
        value = [float(item) for item in value.strip("[]").split(",")]
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return list(default)
    return [float(item) for item in value]


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def finite(*values):
    return all(math.isfinite(float(item)) for item in values)


class GraspComparisonLogger:
    def __init__(self):
        rospy.init_node("grasp_comparison_logger")
        self.vehicle_type = rospy.get_param("~vehicle_type", "iris")
        self.vehicle_id = str(rospy.get_param("~vehicle_id", "0"))
        self.label = rospy.get_param("~label", "run")
        self.output_csv = rospy.get_param("~output_csv", "")
        self.pick_uav_offset = parse_vector(rospy.get_param("~pick_uav_offset", [0.0, 0.0, 0.15]), [0.0, 0.0, 0.15])
        self.enable_workspace_assist = bool(rospy.get_param("~enable_workspace_assist", False))
        self.max_duration = float(rospy.get_param("~max_duration", 180.0))
        fixed_yaw = rospy.get_param("~target_yaw_deg", "")
        self.fixed_yaw = None if str(fixed_yaw).strip() == "" else math.radians(float(fixed_yaw))

        if not self.output_csv:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self.output_csv = os.path.join(
                os.path.expanduser("~"),
                ".ros",
                "grasp_compare_%s_%s.csv" % (self.label, stamp),
            )
        output_dir = os.path.dirname(os.path.abspath(self.output_csv))
        if output_dir and not os.path.isdir(output_dir):
            os.makedirs(output_dir)

        self.local_pose = None
        self.local_velocity = None
        self.bulb_pose = None
        self.mission_state = -1
        self.ready_state = 0
        self.arm_error = [float("nan"), float("nan"), float("nan")]
        self.workspace_correction = [0.0, 0.0, 0.0]
        self.start_time = None
        self.initial_yaw = None
        self.seen_grasp_state = False
        self.rows = 0

        namespace = "/%s_%s" % (self.vehicle_type, self.vehicle_id)
        rospy.Subscriber(namespace + "/mavros/local_position/pose", PoseStamped, self.local_pose_callback, queue_size=1)
        rospy.Subscriber(namespace + "/mavros/local_position/velocity_local", TwistStamped, self.local_velocity_callback, queue_size=1)
        rospy.Subscriber("/light_bulb/body_pose", PoseStamped, self.bulb_pose_callback, queue_size=1)
        rospy.Subscriber("/mission_state", Int32, self.mission_state_callback, queue_size=1)
        rospy.Subscriber("/arm_control_bulb/ready_state", Int32, self.ready_state_callback, queue_size=1)
        rospy.Subscriber("/arm_control_bulb/error", Float64MultiArray, self.arm_error_callback, queue_size=1)
        rospy.Subscriber("/arm_control_bulb/workspace_assist", Float64MultiArray, self.workspace_callback, queue_size=1)

        self.file_obj = open(self.output_csv, "w", newline="")
        self.writer = csv.writer(self.file_obj)
        self.writer.writerow([
            "label",
            "t",
            "mission_state",
            "arm_ready_state",
            "uav_x",
            "uav_y",
            "uav_z",
            "uav_vx",
            "uav_vy",
            "uav_vz",
            "target_x",
            "target_y",
            "target_z",
            "uav_pos_error",
            "uav_xy_error",
            "uav_z_error",
            "uav_yaw_deg",
            "target_yaw_deg",
            "uav_yaw_error_deg",
            "arm_pos_error",
            "arm_axis_error_deg",
            "arm_centerline_error",
        ])
        rospy.loginfo("抓取对比记录器启动: label=%s output=%s", self.label, self.output_csv)

    def local_pose_callback(self, msg):
        self.local_pose = msg
        yaw = yaw_from_quaternion(msg.pose.orientation)
        if self.initial_yaw is None and math.isfinite(yaw):
            self.initial_yaw = yaw

    def local_velocity_callback(self, msg):
        self.local_velocity = msg

    def bulb_pose_callback(self, msg):
        self.bulb_pose = msg

    def mission_state_callback(self, msg):
        self.mission_state = int(msg.data)
        if self.mission_state == 2:
            self.seen_grasp_state = True

    def ready_state_callback(self, msg):
        self.ready_state = int(msg.data)

    def arm_error_callback(self, msg):
        data = list(msg.data)
        while len(data) < 3:
            data.append(float("nan"))
        self.arm_error = [float(data[0]), float(data[1]), float(data[2])]

    def workspace_callback(self, msg):
        if len(msg.data) >= 3 and finite(*msg.data[:3]):
            self.workspace_correction = [float(item) for item in msg.data[:3]]

    def target_position(self):
        if self.bulb_pose is None:
            return None
        correction = self.workspace_correction if self.enable_workspace_assist and self.mission_state in (1, 2, 3) else [0.0, 0.0, 0.0]
        p = self.bulb_pose.pose.position
        return [
            p.x + self.pick_uav_offset[0] + correction[0],
            p.y + self.pick_uav_offset[1] + correction[1],
            p.z + self.pick_uav_offset[2] + correction[2],
        ]

    def desired_yaw(self):
        if self.fixed_yaw is not None:
            return self.fixed_yaw
        return self.initial_yaw if self.initial_yaw is not None else 0.0

    def write_row(self):
        if self.local_pose is None or self.local_velocity is None:
            return
        if self.start_time is None:
            self.start_time = rospy.Time.now()
        t = (rospy.Time.now() - self.start_time).to_sec()
        pose = self.local_pose.pose
        vel = self.local_velocity.twist.linear
        target = self.target_position()
        yaw = yaw_from_quaternion(pose.orientation)
        target_yaw = self.desired_yaw()
        yaw_error = normalize_angle(target_yaw - yaw)

        if target is None:
            target = [float("nan"), float("nan"), float("nan")]
            pos_error = float("nan")
            xy_error = float("nan")
            z_error = float("nan")
        else:
            dx = target[0] - pose.position.x
            dy = target[1] - pose.position.y
            dz = target[2] - pose.position.z
            pos_error = math.sqrt(dx * dx + dy * dy + dz * dz)
            xy_error = math.sqrt(dx * dx + dy * dy)
            z_error = abs(dz)

        self.writer.writerow([
            self.label,
            "%.4f" % t,
            self.mission_state,
            self.ready_state,
            "%.6f" % pose.position.x,
            "%.6f" % pose.position.y,
            "%.6f" % pose.position.z,
            "%.6f" % vel.x,
            "%.6f" % vel.y,
            "%.6f" % vel.z,
            "%.6f" % target[0],
            "%.6f" % target[1],
            "%.6f" % target[2],
            "%.6f" % pos_error,
            "%.6f" % xy_error,
            "%.6f" % z_error,
            "%.6f" % math.degrees(yaw),
            "%.6f" % math.degrees(target_yaw),
            "%.6f" % math.degrees(yaw_error),
            "%.6f" % self.arm_error[0],
            "%.6f" % self.arm_error[1],
            "%.6f" % self.arm_error[2],
        ])
        self.rows += 1

    def should_stop(self):
        if self.start_time is None:
            return False
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        if elapsed > self.max_duration:
            rospy.logwarn("记录达到最大时长 %.1fs，停止", self.max_duration)
            return True
        if self.mission_state == 99:
            rospy.logwarn("任务进入 ABORT，停止记录")
            return True
        if self.seen_grasp_state and (self.ready_state >= 2 or self.mission_state >= 3):
            rospy.loginfo("检测到抓取完成，停止记录")
            return True
        return False

    def run(self):
        rate = rospy.Rate(20)
        try:
            while not rospy.is_shutdown():
                self.write_row()
                if self.should_stop():
                    break
                rate.sleep()
        finally:
            self.file_obj.flush()
            self.file_obj.close()
            rospy.loginfo("抓取对比记录完成: rows=%d output=%s", self.rows, self.output_csv)


if __name__ == "__main__":
    GraspComparisonLogger().run()
