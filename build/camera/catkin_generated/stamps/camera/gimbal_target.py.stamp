#!/usr/bin/env python
# -*- coding: UTF-8 -*- 
import rospy
from math import cos, sin, pi, asin
from camera.msg import servo_msg, pixel_msg, pos_message,gimbal_msg
from geometry_msgs.msg import TransformStamped 
from tf2_geometry_msgs import PointStamped
from mavros_msgs.srv import StreamRate,StreamRateRequest
from sensor_msgs.msg import Imu,NavSatFix
from std_msgs.msg import Float64
import tf2_ros
import tf
import threading
import sys
import os
import time

def gimbal_target_callback(msg):
    """
    回调函数，当接收到新的位置消息时执行。
    该函数将坐标从"World"框架转换到"UAV0"框架，并发布到/gimbal_target话题。
    """
    # 创建一个PointStamped消息，并填充数据
    point = PointStamped()
    point.header.frame_id = "World"  # 输入坐标系
    point.header.stamp = rospy.Time.now()  # 使用ROS时间
    point.point.x = msg.relate_E  # 假设relate_E是东向坐标
    point.point.y = msg.relate_N  # 假设relate_N是北向坐标
    point.point.z = 0  # 假设高度为0，可以根据需要调整

    try:
        # 使用tf2进行坐标变换，从"World"到"UAV0"
        transformed_point = tf_buffer.transform(point, "GIM0", rospy.Duration(1.0))
        # rospy.loginfo(f"Transformed point: {transformed_point}")

        # 创建并填充gimbal_msg消息
        gim_msg = gimbal_msg()
        gim_msg.pos_x = transformed_point.point.x
        gim_msg.pos_y = transformed_point.point.y
        gim_msg.height = transformed_point.point.z
        # 根据gimbal_msg的定义，填充其他必要的字段
        # 例如，如果有角度或其他控制参数，可以在这里设置

        # 发布到/gimbal_target话题
        gimbal_target_pub.publish(gim_msg)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"Transform failed: {e}")

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('np_gimbal_target', anonymous=True)

    # 初始化tf2缓冲区和监听器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 等待一段时间以确保tf2缓冲区接收到足够的变换信息
    rospy.sleep(1.0)

    # 发布舵机控制指令到/gimbal_target话题
    gimbal_target_pub = rospy.Publisher("/gimbal_target", gimbal_msg, queue_size=10)

    # 订阅接收到的位置消息
    sub_pos = rospy.Subscriber("/pos_message", pos_message, gimbal_target_callback)

    # 保持节点运行，等待回调
    rospy.spin()