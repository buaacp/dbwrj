#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, NavSatFix
from camera.msg import pixel_msg, servo_msg, pos_message
import time

def generate_fake_data():
    """生成伪数据，用于模拟传感器和相机的输入。"""
    # 模拟相对高度
    relative_altitude = Float64()
    relative_altitude.data = random.uniform(0.9, 1) # 假设飞行高度为0到1000米之间
    
    # 模拟IMU数据 (例如，旋转角度)
    imu_data = Imu()
    imu_data.orientation.x = 0
    imu_data.orientation.y = 0
    imu_data.orientation.z = 0
    imu_data.orientation.w = 1
    
    # 模拟GPS坐标
    gps_data = NavSatFix()
    gps_data.latitude = random.uniform(-90, 90)
    gps_data.longitude = random.uniform(-180, 180)
    gps_data.altitude = random.uniform(0, 5000)  # 假设海拔高度为0到5000米之间
    
    # 模拟像素位置
    pixel_data = pixel_msg()
    pixel_data.pixel_x = random.randint(0, 640)  # 假设图像宽度为640
    pixel_data.pixel_y = random.randint(0, 480)  # 假设图像高度为480
    pixel_data.time = rospy.Time.now()
    
    # 模拟伺服角度
    servo_data = servo_msg()
    servo_data.angle1 = random.uniform(-0.09, 0.09)  # 假设偏航角为-90到90度
    servo_data.angle2 = random.uniform(-0.09, 0.09)  # 假设俯仰角为-90到90度
    
    return relative_altitude, imu_data, gps_data, pixel_data, servo_data

def publish_fake_data():
    """发布伪数据以模拟回调函数接收数据。"""
    # 初始化ROS节点
    rospy.init_node('simulation_publisher', anonymous=True)
    
    # 发布者
    relative_altitude_pub = rospy.Publisher('mavros/global_position/rel_alt', Float64, queue_size=10)
    imu_pub = rospy.Publisher('mavros/imu/data', Imu, queue_size=10)
    gps_pub = rospy.Publisher('mavros/global_position/global', NavSatFix, queue_size=10)
    servo_pub = rospy.Publisher('/servo_angles', servo_msg, queue_size=10)
    
    rate = rospy.Rate(50)  # 设定发布频率为20Hz
    
    while not rospy.is_shutdown():
        # 生成伪数据
        relative_altitude, imu_data, gps_data, pixel_data, servo_data = generate_fake_data()
        
        # 发布数据
        relative_altitude_pub.publish(relative_altitude)
        imu_pub.publish(imu_data)
        gps_pub.publish(gps_data)
        servo_pub.publish(servo_data)
        
        # 等待
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_fake_data()
    except rospy.ROSInterruptException:
        pass
