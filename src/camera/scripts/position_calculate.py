#! /usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import numpy as np
import time
from math import cos, sin, tan, sqrt, pi, atan, asin
from camera.msg import servo_msg


import threading

# 消息定义
from std_msgs.msg import Float64
from camera.msg import pixel_msg
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import PointStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from camera.msg import pos_message
from mavros_msgs.srv import StreamRate,StreamRateRequest

# 类
import sys
import os
current_dir = os.path.dirname(__file__)  # 获取当前脚本所在目录
sys.path.append(current_dir)  # 将当前目录加入到 sys.path
import CAMERA

# tf
import tf2_ros
import tf

servo_angle_msg=servo_msg()
servo_angle_msg.angle1 = 0
servo_angle_msg.angle2 = 0

camera_state=CAMERA.Mycamera()
pic_width = 640
pic_height = 480
pic_long = 528

EARTH_RADIUS=6378.137 #地球半径 KM

def UAV_info_update():
    relative_altitude_sub = rospy.Subscriber('mavros/global_position/rel_alt', Float64, relative_altitude_callback)
    attitude_sub = rospy.Subscriber('mavros/imu/data', Imu, attitude_callback)
    global_position_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, global_position_callback)
    servo_angle_sub = rospy.Subscriber('/servo_angle', servo_msg, servo_angle_callback)
    
    rospy.spin()

def relative_altitude_callback(msg):
    camera_state.relative_altitude_info_callback(msg)
# 姿态回调
def attitude_callback(msg):
    camera_state.attitude_info_callback(msg)

def global_position_callback(msg):
    camera_state.global_position_callback(msg)

def servo_angle_callback(msg):
    servo_angle_msg=msg



def BROADCAST_WORLD_UAV():
    while not rospy.is_shutdown():
        broadcaster_world_UAV = tf2_ros.TransformBroadcaster()
        #     无人机到世界坐标系
        tfs = TransformStamped()
        tfs.header.frame_id = "World"
        tfs.header.stamp = rospy.Time.now()

        tfs.child_frame_id = "UAV0"
        tfs.transform.translation.x = 0.0
        tfs.transform.translation.y = 0.0
        tfs.transform.translation.z = camera_state.altitude #飞行高度
        tfs.transform.rotation.x = camera_state.UAV_x
        tfs.transform.rotation.y = camera_state.UAV_y
        tfs.transform.rotation.z = camera_state.UAV_z
        tfs.transform.rotation.w = camera_state.UAV_w

        broadcaster_world_UAV.sendTransform(tfs)

        time.sleep(0.02)

def position_calculate_callback(pixel_pos):

    pos_x = pic_height/2 - pixel_pos.pixel_y  
    pos_y = pic_width/2  - pixel_pos.pixel_x
    pos_z = -1*pic_long

    # 解算位置 
    global buffer
    global listener
    global pub_pos
    global pos_msg
    time.sleep(0.1)


    # 创建一依赖于 CAM0 的坐标点，调用 API 求出该点在 world 中的坐标
    point_source = PointStamped()
    point_source.header.frame_id = "CAM0"
    point_source.header.stamp = pixel_pos.time
    point_source.point.x = pos_x
    point_source.point.y = pos_y
    point_source.point.z = pos_z
    # 创建一依赖于 camera 的坐标原点，调用 API 求出该点在 world 中的坐标
    point_source0 = PointStamped()
    point_source0.header.frame_id = "CAM0"
    point_source0.header.stamp = pixel_pos.time
    point_source0.point.x = 0
    point_source0.point.y = 0
    point_source0.point.z = 0


    point_target  = buffer.transform(point_source,"World")
    point_target0 = buffer.transform(point_source0,"World")

    if point_target.point.z<point_target0.point.z:
        prey_x=point_target.point.x/(point_target0.point.z-point_target.point.z)*point_target0.point.z
        prey_y=point_target.point.y/(point_target0.point.z-point_target.point.z)*point_target0.point.z
        rospy.loginfo("坐标点相对于 world 的坐标:(E: %.6f N: %.6f)",prey_x,prey_y)

        obj_lat = camera_state.latitude + asin(prey_y/2/1000/EARTH_RADIUS)*2*180/pi
        obj_lon = camera_state.longitude + asin(prey_x/EARTH_RADIUS/1000/cos(camera_state.latitude*pi/180)/2)*2*180/pi
        rospy.loginfo("坐标点经纬度坐标:(E: %.6f N: %.6f)",obj_lon,obj_lat)

        pos_msg.time     = pixel_pos.time
        pos_msg.relate_E = prey_x
        pos_msg.relate_N = prey_y
        pos_msg.tar_lat  = obj_lat
        pos_msg.tar_lon  = obj_lon

        pub_pos.publish(pos_msg)






if __name__ == "__main__":
    rospy.init_node('np_Position', anonymous=True)
    #修改参数服务器 
    set_stream_rate = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)

    # 创建StreamRate请求对象
    stream_rate_request = StreamRateRequest()
    set_stream_rate.wait_for_service()
    
    # 设置请求参数
    stream_rate_request.stream_id = 0  # 设置数据流ID
    stream_rate_request.message_rate = 20  # 设置数据流速率，这里设置为10Hz
    stream_rate_request.on_off = True  # 设置数据流是否开启

    # 调用服务
    try:
        result = set_stream_rate.call(stream_rate_request)
        rospy.loginfo("Set stream rate successful: %s", result)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

    time.sleep(2)


    global pub_pos
    pub_pos = rospy.Publisher("/pos_message",pos_message,queue_size=10)
    global pos_msg
    pos_msg=pos_message()
    # 启动线程回调高度
    thread_Sub = threading.Thread(target=UAV_info_update)
    thread_Sub.start()

    thread_UAV_WORLD = threading.Thread(target=BROADCAST_WORLD_UAV)
    thread_UAV_WORLD.start()

    #相机对于无人机坐标系
    broadcaster_UAV_CAM = tf2_ros.StaticTransformBroadcaster()
    tfs_UAV_CAM = TransformStamped()
    # --- 头信息
    tfs_UAV_CAM.header.frame_id = "UAV0"
    tfs_UAV_CAM.header.stamp = rospy.Time.now()
    # --- 子坐标系
    tfs_UAV_CAM.child_frame_id = "CAM0"
    # --- 坐标系相对信息
    # ------ 偏移量
    tfs_UAV_CAM.transform.translation.x = 0.0
    tfs_UAV_CAM.transform.translation.y = 0.0
    tfs_UAV_CAM.transform.translation.z = -0.20  #camera相对于飞控位置
    # ------ 四元数
    qtn = tf.transformations.quaternion_from_euler(0,0,0) #相机放置角度
    tfs_UAV_CAM.transform.rotation.x = qtn[0]
    tfs_UAV_CAM.transform.rotation.y = qtn[1]
    tfs_UAV_CAM.transform.rotation.z = qtn[2]
    tfs_UAV_CAM.transform.rotation.w = qtn[3]
    # 广播器发送消息
    broadcaster_UAV_CAM.sendTransform(tfs_UAV_CAM)
    # 创建 TF 订阅对象
    global buffer
    global listener
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    
    detect_info_sub = rospy.Subscriber("/detect", pixel_msg,position_calculate_callback)
    rospy.spin()

