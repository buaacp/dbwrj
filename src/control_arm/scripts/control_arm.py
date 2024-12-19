#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

def control_joint_positions():
    rospy.init_node('control_joint_positions_demo')

    # 创建Action客户端，连接到控制器的Action服务器
    client = actionlib.SimpleActionClient('/le_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("等待控制器的Action服务器...")
    client.wait_for_server()
    rospy.loginfo("控制器的Action服务器已连接")

    # 定义关节名称
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                   'elbow_joint', 'wrist_1_joint', 'wrist_2_joint']

    # 创建轨迹消息
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names

    # 创建轨迹点
    point = JointTrajectoryPoint()
    # 设置每个关节的目标位置（单位：弧度）
    point.positions = [0.5, -0.5, 0.3, -0.3, 0.2]
    # 设置到达目标位置的时间
    point.time_from_start = rospy.Duration(2.0)  # 2秒内到达目标

    # 将轨迹点添加到轨迹中
    trajectory.points.append(point)

    # 创建FollowJointTrajectoryGoal消息
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory

    # 发送目标
    client.send_goal(goal)
    rospy.loginfo("已发送轨迹目标，等待结果...")

    # 等待结果
    client.wait_for_result()
    rospy.loginfo("运动完成")

if __name__ == '__main__':
    try:
        control_joint_positions()
    except rospy.ROSInterruptException:
        pass

