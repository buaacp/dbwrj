#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
import tf2_ros
import geometry_msgs.msg
import math

def imu_callback(msg):
    # 创建 TF 消息
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "camera_link"

    # 假设我们直接使用 IMU 提供的 orientation 四元数
    t.transform.rotation = msg.orientation

    # 如果需要位置，可设置为 0
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0

    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("d435i_imu_tf")
    br = tf2_ros.TransformBroadcaster()
    
    # 订阅 IMU
    rospy.Subscriber("/imu/data_raw", Imu, imu_callback)

    rospy.spin()
