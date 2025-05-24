#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from gazebo_msgs.msg import ModelStates

# 原有无人机配置
vehicle_type = 'iris'
vehicle_num = 1
multi_pose_pub = [None] * vehicle_num
multi_speed_pub = [None] * vehicle_num
multi_local_pose = [PoseStamped() for _ in range(vehicle_num)]
multi_speed = [Vector3Stamped() for _ in range(vehicle_num)]

# 新增 weightless_ball 配置
ball_name = 'weightless_ball'  # Gazebo 中球的模型名称
ball_pose_pub = None
ball_speed_pub = None
ball_pose = PoseStamped()
ball_speed = Vector3Stamped()

def gazebo_model_state_callback(msg):
    # 处理无人机数据（原有逻辑）
    for vehicle_id in range(vehicle_num):
        model_name = "{}_{}".format(vehicle_type, vehicle_id)
        try:
            model_idx = msg.name.index(model_name)
        except ValueError:
            rospy.logwarn_throttle(10, "Model %s not found!" % model_name)
            continue

        # 更新无人机位姿和速度
        multi_local_pose[vehicle_id].header.stamp = rospy.Time.now()
        multi_local_pose[vehicle_id].header.frame_id = 'map'
        multi_local_pose[vehicle_id].pose = msg.pose[model_idx]

        multi_speed[vehicle_id].header.stamp = rospy.Time.now()
        multi_speed[vehicle_id].header.frame_id = 'map'
        multi_speed[vehicle_id].vector = msg.twist[model_idx]

    # 新增 weightless_ball 处理
    try:
        ball_idx = msg.name.index(ball_name)
    except ValueError:
        rospy.logwarn_throttle(10, "Model %s not found!" % ball_name)
        return

    # 更新球的位姿和速度
    ball_pose.header.stamp = rospy.Time.now()
    ball_pose.header.frame_id = 'map'
    ball_pose.pose = msg.pose[ball_idx]

    ball_speed.header.stamp = rospy.Time.now()
    ball_speed.header.frame_id = 'map'
    ball_speed.vector = msg.twist[ball_idx]

if __name__ == '__main__':
    rospy.init_node("%s_get_pose_groundtruth" % vehicle_type)
    
    # 订阅 Gazebo 模型状态（原有）
    gazebo_model_state_sub = rospy.Subscriber(
        "/gazebo/model_states", ModelStates, 
        gazebo_model_state_callback, queue_size=1
    )

    # 初始化无人机发布器（原有）
    for i in range(vehicle_num):
        multi_pose_pub[i] = rospy.Publisher(
            "{}_{}/mavros/vision_pose/pose".format(vehicle_type, i), 
            PoseStamped, queue_size=1
        )
        multi_speed_pub[i] = rospy.Publisher(
            "{}_{}/mavros/vision_speed/speed".format(vehicle_type, i), 
            Vector3Stamped, queue_size=1
        )
        print("Get {}_{} groundtruth pose".format(vehicle_type, i))

    # 新增 weightless_ball 发布器
    ball_pose_pub = rospy.Publisher(
        "/weightless_ball/pose", PoseStamped, queue_size=1
    )
    ball_speed_pub = rospy.Publisher(
        "/weightless_ball/speed", Vector3Stamped, queue_size=1
    )
    print("Get %s groundtruth pose" % ball_name)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # 发布无人机数据（原有）
        for i in range(vehicle_num):
            multi_pose_pub[i].publish(multi_local_pose[i])
            multi_speed_pub[i].publish(multi_speed[i])
        
        # 新增发布球的数据
        ball_pose_pub.publish(ball_pose)
        ball_speed_pub.publish(ball_speed)

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            continue