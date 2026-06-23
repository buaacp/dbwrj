#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped, Vector3Stamped
from gazebo_msgs.msg import LinkStates, ModelStates
from mavros_msgs.msg import GPSINPUT
import math

# 原有无人机配置
vehicle_type = 'iris'
vehicle_num = 1
multi_pose_pub = [None] * vehicle_num
multi_speed_pub = [None] * vehicle_num
multi_speed_twist_cov_pub = [None] * vehicle_num
multi_mocap_pose_pub = [None] * vehicle_num
multi_fake_gps_mocap_pose_pub = [None] * vehicle_num
multi_gps_input_pub = [None] * vehicle_num
multi_local_pose = [PoseStamped() for _ in range(vehicle_num)]
multi_speed = [Vector3Stamped() for _ in range(vehicle_num)]
multi_pose_ready = [False] * vehicle_num

gps_origin_lat = 47.397742
gps_origin_lon = 8.545594
gps_origin_alt = 488.0
earth_radius_m = 6378137.0

# 新增 weightless_ball 配置
ball_name = 'weightless_ball'  # Gazebo 中球的模型名称
ball_pose_pub = None
ball_speed_pub = None
ball_pose = PoseStamped()
ball_speed = Vector3Stamped()
ball_ready = False

# 新增灯泡和灯座配置
light_bulb_name = 'light_bulb'
light_bulb_fixture_name = 'light_bulb_fixture'
light_bulb_body_link = 'light_bulb::bulb_body_link'
socket_link = 'light_bulb_fixture::socket_base_link'

light_bulb_pose_pub = None
light_bulb_speed_pub = None
light_bulb_body_pose_pub = None
light_bulb_fixture_pose_pub = None
socket_pose_pub = None

light_bulb_pose = PoseStamped()
light_bulb_speed = Vector3Stamped()
light_bulb_body_pose = PoseStamped()
light_bulb_fixture_pose = PoseStamped()
socket_pose = PoseStamped()
light_bulb_ready = False
light_bulb_fixture_ready = False
light_bulb_body_ready = False
socket_ready = False

def gazebo_model_state_callback(msg):
    global ball_ready, light_bulb_ready, light_bulb_fixture_ready
    now = rospy.Time.now()

    # 处理无人机数据（原有逻辑）
    for vehicle_id in range(vehicle_num):
        model_name = "{}_{}".format(vehicle_type, vehicle_id)
        try:
            model_idx = msg.name.index(model_name)
        except ValueError:
            rospy.logwarn_throttle(10, "Model %s not found!" % model_name)
            continue

        # 更新无人机位姿和速度
        multi_local_pose[vehicle_id].header.stamp = now
        multi_local_pose[vehicle_id].header.frame_id = 'map'
        multi_local_pose[vehicle_id].pose = msg.pose[model_idx]

        multi_speed[vehicle_id].header.stamp = now
        multi_speed[vehicle_id].header.frame_id = 'map'
        multi_speed[vehicle_id].vector = msg.twist[model_idx].linear
        multi_pose_ready[vehicle_id] = True

    # 新增 weightless_ball 处理
    try:
        ball_idx = msg.name.index(ball_name)
    except ValueError:
        rospy.logwarn_throttle(10, "Model %s not found!" % ball_name)
    else:
        # 更新球的位姿和速度
        ball_pose.header.stamp = now
        ball_pose.header.frame_id = 'map'
        ball_pose.pose = msg.pose[ball_idx]

        ball_speed.header.stamp = now
        ball_speed.header.frame_id = 'map'
        ball_speed.vector = msg.twist[ball_idx].linear
        ball_ready = True

    # 新增 light_bulb 处理
    try:
        bulb_idx = msg.name.index(light_bulb_name)
    except ValueError:
        rospy.logwarn_throttle(10, "Model %s not found!" % light_bulb_name)
    else:
        light_bulb_pose.header.stamp = now
        light_bulb_pose.header.frame_id = 'map'
        light_bulb_pose.pose = msg.pose[bulb_idx]

        light_bulb_speed.header.stamp = now
        light_bulb_speed.header.frame_id = 'map'
        light_bulb_speed.vector = msg.twist[bulb_idx].linear
        light_bulb_ready = True

    # 新增 light_bulb_fixture 处理
    try:
        fixture_idx = msg.name.index(light_bulb_fixture_name)
    except ValueError:
        rospy.logwarn_throttle(10, "Model %s not found!" % light_bulb_fixture_name)
    else:
        light_bulb_fixture_pose.header.stamp = now
        light_bulb_fixture_pose.header.frame_id = 'map'
        light_bulb_fixture_pose.pose = msg.pose[fixture_idx]
        light_bulb_fixture_ready = True

def gazebo_link_state_callback(msg):
    global light_bulb_body_ready, socket_ready
    now = rospy.Time.now()

    # 新增灯泡本体 link 处理
    try:
        bulb_body_idx = msg.name.index(light_bulb_body_link)
    except ValueError:
        rospy.logwarn_throttle(10, "Link %s not found!" % light_bulb_body_link)
    else:
        light_bulb_body_pose.header.stamp = now
        light_bulb_body_pose.header.frame_id = 'map'
        light_bulb_body_pose.pose = msg.pose[bulb_body_idx]
        light_bulb_body_ready = True

    # 新增灯座 socket link 处理
    try:
        socket_idx = msg.name.index(socket_link)
    except ValueError:
        rospy.logwarn_throttle(10, "Link %s not found!" % socket_link)
    else:
        socket_pose.header.stamp = now
        socket_pose.header.frame_id = 'map'
        socket_pose.pose = msg.pose[socket_idx]
        socket_ready = True

if __name__ == '__main__':
    rospy.init_node("%s_get_pose_groundtruth" % vehicle_type)
    
    # 订阅 Gazebo 模型状态（原有）
    gazebo_model_state_sub = rospy.Subscriber(
        "/gazebo/model_states", ModelStates, 
        gazebo_model_state_callback, queue_size=1
    )
    gazebo_link_state_sub = rospy.Subscriber(
        "/gazebo/link_states", LinkStates,
        gazebo_link_state_callback, queue_size=1
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
        multi_speed_twist_cov_pub[i] = rospy.Publisher(
            "{}_{}/mavros/vision_speed/speed_twist_cov".format(vehicle_type, i),
            TwistWithCovarianceStamped, queue_size=1
        )
        multi_mocap_pose_pub[i] = rospy.Publisher(
            "{}_{}/mavros/mocap/pose".format(vehicle_type, i),
            PoseStamped, queue_size=1
        )
        multi_fake_gps_mocap_pose_pub[i] = rospy.Publisher(
            "{}_{}/mavros/fake_gps/mocap/pose".format(vehicle_type, i),
            PoseStamped, queue_size=1
        )
        multi_gps_input_pub[i] = rospy.Publisher(
            "{}_{}/mavros/gps_input/gps_input".format(vehicle_type, i),
            GPSINPUT, queue_size=1
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

    # 新增灯泡和灯座发布器
    light_bulb_pose_pub = rospy.Publisher(
        "/light_bulb/pose", PoseStamped, queue_size=1
    )
    light_bulb_speed_pub = rospy.Publisher(
        "/light_bulb/speed", Vector3Stamped, queue_size=1
    )
    light_bulb_body_pose_pub = rospy.Publisher(
        "/light_bulb/body_pose", PoseStamped, queue_size=1
    )
    light_bulb_fixture_pose_pub = rospy.Publisher(
        "/light_bulb_fixture/pose", PoseStamped, queue_size=1
    )
    socket_pose_pub = rospy.Publisher(
        "/light_bulb_fixture/socket_pose", PoseStamped, queue_size=1
    )
    print("Get %s and %s groundtruth pose" % (light_bulb_name, light_bulb_fixture_name))

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # 发布无人机数据（原有）
        for i in range(vehicle_num):
            if not multi_pose_ready[i]:
                continue
            multi_pose_pub[i].publish(multi_local_pose[i])
            multi_speed_pub[i].publish(multi_speed[i])
            speed_twist = TwistWithCovarianceStamped()
            speed_twist.header = multi_speed[i].header
            speed_twist.twist.twist.linear.x = multi_speed[i].vector.x
            speed_twist.twist.twist.linear.y = multi_speed[i].vector.y
            speed_twist.twist.twist.linear.z = multi_speed[i].vector.z
            speed_twist.twist.covariance[0] = 0.01
            speed_twist.twist.covariance[7] = 0.01
            speed_twist.twist.covariance[14] = 0.01
            speed_twist.twist.covariance[21] = 0.01
            speed_twist.twist.covariance[28] = 0.01
            speed_twist.twist.covariance[35] = 0.01
            multi_speed_twist_cov_pub[i].publish(speed_twist)
            multi_mocap_pose_pub[i].publish(multi_local_pose[i])
            multi_fake_gps_mocap_pose_pub[i].publish(multi_local_pose[i])

            gps = GPSINPUT()
            gps.header = multi_local_pose[i].header
            gps.fix_type = GPSINPUT.GPS_FIX_TYPE_3D_FIX
            gps.gps_id = i
            gps.ignore_flags = 0
            gps.time_week_ms = 0
            gps.time_week = 0
            lat = gps_origin_lat + (
                multi_local_pose[i].pose.position.y / earth_radius_m
            ) * 180.0 / math.pi
            lon = gps_origin_lon + (
                multi_local_pose[i].pose.position.x /
                (earth_radius_m * math.cos(math.radians(gps_origin_lat)))
            ) * 180.0 / math.pi
            gps.lat = int(lat * 1e7)
            gps.lon = int(lon * 1e7)
            gps.alt = gps_origin_alt + multi_local_pose[i].pose.position.z
            gps.hdop = 0.7
            gps.vdop = 0.7
            # Gazebo/MAVROS pose is ENU; GPS_INPUT velocity is NED.
            gps.vn = multi_speed[i].vector.y
            gps.ve = multi_speed[i].vector.x
            gps.vd = -multi_speed[i].vector.z
            gps.speed_accuracy = 0.05
            gps.horiz_accuracy = 0.5
            gps.vert_accuracy = 0.5
            gps.satellites_visible = 10
            gps.yaw = 0
            multi_gps_input_pub[i].publish(gps)
        
        # 新增发布球的数据
        if ball_ready:
            ball_pose_pub.publish(ball_pose)
            ball_speed_pub.publish(ball_speed)

        # 新增发布灯泡和灯座数据
        if light_bulb_ready:
            light_bulb_pose_pub.publish(light_bulb_pose)
            light_bulb_speed_pub.publish(light_bulb_speed)
        if light_bulb_body_ready:
            light_bulb_body_pose_pub.publish(light_bulb_body_pose)
        if light_bulb_fixture_ready:
            light_bulb_fixture_pose_pub.publish(light_bulb_fixture_pose)
        if socket_ready:
            socket_pose_pub.publish(socket_pose)

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            continue
