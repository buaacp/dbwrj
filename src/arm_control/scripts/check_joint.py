#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from sensor_msgs.msg import JointState

# 全局变量
velocity = []

def joint_states_callback(data):
    """回调函数，处理接收到的关节状态消息"""
    global velocity
    velocity = data.velocity
    print(velocity)

def joint_states_listener():
    """订阅 /joint_states 话题"""
    rospy.init_node('joint_states_listener', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)
    rospy.loginfo("Listening to /joint_states topic...")
    rospy.spin()  # 保持节点运行，直到被手动停止

if __name__ == '__main__':
    try:
        joint_states_listener()
    except rospy.ROSInterruptException:
        pass