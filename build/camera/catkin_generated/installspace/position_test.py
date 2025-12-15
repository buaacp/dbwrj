#!/usr/bin/env python2
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
import math
import json

# 添加当前脚本所在目录到sys.path
current_dir = os.path.dirname(__file__)
sys.path.append(current_dir)
import CAMERA  # 确保CAMERA模块存在且正确

# 常量定义
PIC_WIDTH = 640
PIC_HEIGHT = 480
PIC_LONG = 539
EARTH_RADIUS = 6378.137  # 地球半径 (KM)


class NPPositionNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('np_Position', anonymous=True)

        self.camera_state=CAMERA.Mycamera()

        if_simulation = rospy.get_param('/if_simulation', False)

        self.current_servo_angle1 = 0.0  # 初始角度1（偏航）
        self.current_servo_angle2 = 0.0  # 初始角度2（俯仰）

        # 初始化TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 发布UAV和World_uav坐标转换
        self.broadcaster_world_UAV_ = tf2_ros.TransformBroadcaster()
        self.broadcaster_world_UAV = tf2_ros.TransformBroadcaster()

        if not if_simulation:
            print("need change rate")
            #修改参数服务器 
            set_stream_rate = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)

            # 创建StreamRate请求对象
            stream_rate_request = StreamRateRequest()
            set_stream_rate.wait_for_service()
            
            # 设置请求参数
            stream_rate_request.stream_id = 0  # 设置数据流ID
            stream_rate_request.message_rate = 20  # 设置数据流速率，这里设置为10Hz
            stream_rate_request.on_off = True  # 设置数据流是否开启

            # # 调用服务
            try:
                result = set_stream_rate.call(stream_rate_request)
                rospy.loginfo("Set stream rate successful: %s", result)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)
            time.sleep(2)

        # 启动线程低频率发布静态坐标
        self.broadcaster_world_worlduav = tf2_ros.TransformBroadcaster()
        self.broadcaster_GIM2_CAM = tf2_ros.TransformBroadcaster()
        self.broadcaster_uav_gim0 = tf2_ros.TransformBroadcaster()


        # 发布者
        self.pub_pos = rospy.Publisher("/pos_message", pos_message, queue_size=10)
        # 初始化pos_message
        self.pos_msg = pos_message()
        
        # 启动广播GIM1和GIM2到UAV0和GIM1的动态变换
        self.broadcaster_gim = tf2_ros.TransformBroadcaster()
        

    def UAV_info_update(self):
        relative_altitude_sub = rospy.Subscriber('mavros/global_position/rel_alt', Float64, self.relative_altitude_callback)
        attitude_sub = rospy.Subscriber('mavros/imu/data', Imu, self.attitude_callback)
        global_position_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.global_position_callback)

    
    def Camera_info_update(self):
        # 订阅像素位置主题 /detect
        detect_info_sub = rospy.Subscriber("/detect", pixel_msg, self.position_calculate_callback)
        # 订阅云台角度主题 /servo_angles
        servo_sub = rospy.Subscriber("/servo_angles", servo_msg, self.servo_callback)

    def Static_info_pub(self):
        rate = rospy.Rate(50)  # 设置频率为50Hz

        while not rospy.is_shutdown():
            # 广播world到world_uav的静态变换
            self.broadcast_static_world_worlduav()
            # 广播world_uav到uav0的静态变换
            self.pub_world_uav(self.broadcaster_world_UAV_)
            # 广播uav0到gim0的静态变换
            self.broadcast_static_uav_gim0()
            # 广播GIM0到GIM2
            self.broadcast_static_gim0_gim2()
            # 广播相机到GIM2的静态变换
            self.broadcast_static_gim2_cam()
            # 在每次循环中处理一个回调
            
            # 等待下一轮循环
            rate.sleep()



    def relative_altitude_callback(self,msg):
        self.camera_state.relative_altitude_info_callback(msg)
    # 姿态回调
    def attitude_callback(self,msg):
        self.camera_state.attitude_info_callback(msg)
        # print("pub_world_uav_attitude")
        # self.pub_world_uav(self.broadcaster_world_UAV)

    def pub_world_uav(self, puber):
        # 发布 world_uav2uav
        tfs = TransformStamped()
        tfs.header.stamp = rospy.Time.now()
        tfs.header.frame_id = "World_uav"
        tfs.child_frame_id = "UAV0"
        
        # 设置无人机的位置
        tfs.transform.translation.x = 0.0
        tfs.transform.translation.y = 0.0
        tfs.transform.translation.z = self.camera_state.altitude
        
        # 设置无人机的旋转（四元数）
        tfs.transform.rotation.x = self.camera_state.UAV_x
        tfs.transform.rotation.y = self.camera_state.UAV_y
        tfs.transform.rotation.z = self.camera_state.UAV_z
        tfs.transform.rotation.w = self.camera_state.UAV_w

        puber.sendTransform(tfs)
        # rospy.loginfo("Broadcasted transform from World_uav to UAV0")

    def global_position_callback(self,msg):
        self.camera_state.global_position_callback(msg)
    def fakegps_callback(self,msg):
        self.camera_state.global_position_callback(msg)

    def broadcast_static_uav_gim0(self):
        tfs_gim0 = TransformStamped()
        tfs_gim0.header.stamp = rospy.Time.now()
        tfs_gim0.header.frame_id = "UAV0"
        tfs_gim0.child_frame_id = "GIM0"
        
        # 设置云台的位置
        tfs_gim0.transform.translation.x = 0.0
        tfs_gim0.transform.translation.y = 0.15
        tfs_gim0.transform.translation.z = -0.05
        
        # 设置无人机的旋转（四元数）
        tfs_gim0.transform.rotation.x = 0
        tfs_gim0.transform.rotation.y = 0
        tfs_gim0.transform.rotation.z = 0
        tfs_gim0.transform.rotation.w = 1
        self.broadcaster_uav_gim0.sendTransform(tfs_gim0)

    def broadcast_static_world_worlduav(self):
        tfs_world = TransformStamped()
        tfs_world.header.stamp = rospy.Time.now()
        tfs_world.header.frame_id = "World"
        tfs_world.child_frame_id = "World_uav"
        
        # 设置新世界的位置
        tfs_world.transform.translation.x = 0.0
        tfs_world.transform.translation.y = 0.0
        tfs_world.transform.translation.z = 0.0
        # 顺时针旋转90度，角度为pi/2，单位向量沿Z轴

        # 设置无人机世界的旋转（四元数）
        tfs_world.transform.rotation.x = 0
        tfs_world.transform.rotation.y = 0
        tfs_world.transform.rotation.z = -1*math.sqrt(2)/2
        tfs_world.transform.rotation.w = 1*math.sqrt(2)/2
        
        self.broadcaster_world_worlduav.sendTransform(tfs_world)
        # rospy.loginfo("Broadcasted transform from World to World_uav")
        
    def broadcast_static_gim2_cam(self):
        tfs_GIM2_CAM = TransformStamped()
        tfs_GIM2_CAM.header.stamp = rospy.Time.now()
        tfs_GIM2_CAM.header.frame_id = "GIM2"
        tfs_GIM2_CAM.child_frame_id = "CAM0"
        
        # 设置相机相对于GIM2的偏移量
        tfs_GIM2_CAM.transform.translation.x = 0.0
        tfs_GIM2_CAM.transform.translation.y = 0.0
        tfs_GIM2_CAM.transform.translation.z = -0.02  # 相机相对于GIM2的位置
        
        # 设置相机的旋转（四元数），假设相机固定放置
        qtn = tf.transformations.quaternion_from_euler(0, 0, 0)  # 相机放置角度
        tfs_GIM2_CAM.transform.rotation.x = qtn[0]
        tfs_GIM2_CAM.transform.rotation.y = qtn[1]
        tfs_GIM2_CAM.transform.rotation.z = qtn[2]
        tfs_GIM2_CAM.transform.rotation.w = qtn[3]
        
        # 发送静态变换
        self.broadcaster_GIM2_CAM.sendTransform(tfs_GIM2_CAM)

    def broadcast_static_gim0_gim2(self):
        angle1 = self.current_servo_angle1  # 伺服器角度1（偏航）
        angle2 = self.current_servo_angle2  # 伺服器角度2（俯仰）
        
        # 将角度转换为四元数
        # 假设 angle1 是绕Z轴旋转（偏航），angle2 是绕X轴旋转（俯仰）
        qtn_gim1 = tf.transformations.quaternion_from_euler(0, 0, angle1)
        qtn_gim2 = tf.transformations.quaternion_from_euler(angle2,0, 0)
        
        # 广播GIM1相对于UAV0的变换
        tfs_gim1 = TransformStamped()
        tfs_gim1.header.stamp = rospy.Time.now()
        tfs_gim1.header.frame_id = "GIM0"
        tfs_gim1.child_frame_id = "GIM1"
        
        # 设置GIM1的位置相对于无人机，这里假设GIM1与UAV0重合
        tfs_gim1.transform.translation.x = 0.0
        tfs_gim1.transform.translation.y = 0.0
        tfs_gim1.transform.translation.z = -0.0619  # 根据实际情况调整
        
        # 设置GIM1的旋转（四元数）
        tfs_gim1.transform.rotation.x = qtn_gim1[0]
        tfs_gim1.transform.rotation.y = qtn_gim1[1]
        tfs_gim1.transform.rotation.z = qtn_gim1[2]
        tfs_gim1.transform.rotation.w = qtn_gim1[3]
        
        # 广播GIM2相对于GIM1的变换
        tfs_gim2 = TransformStamped()
        tfs_gim2.header.stamp = rospy.Time.now()
        tfs_gim2.header.frame_id = "GIM1"
        tfs_gim2.child_frame_id = "GIM2"
        
        # 设置GIM2的位置相对于GIM1，这里假设GIM2与GIM1重合
        tfs_gim2.transform.translation.x = 0.0
        tfs_gim2.transform.translation.y = 0.0
        tfs_gim2.transform.translation.z = 0.0  # 根据实际情况调整
        
        # 设置GIM2的旋转（四元数）
        tfs_gim2.transform.rotation.x = qtn_gim2[0]
        tfs_gim2.transform.rotation.y = qtn_gim2[1]
        tfs_gim2.transform.rotation.z = qtn_gim2[2]
        tfs_gim2.transform.rotation.w = qtn_gim2[3]
        
        # 一次性广播两个变换
        self.broadcaster_gim.sendTransform([tfs_gim1, tfs_gim2])
        # rospy.loginfo("更新云台角度: gim1 (Yaw)=%.2f, gim2 (Pitch)=%.2f", self.current_servo_angle1, self.current_servo_angle2)
    
    
    def servo_callback(self, servo_msg_data):
        # 更新当前的伺服角度
        self.current_servo_angle1 = servo_msg_data.angle1  # 偏航角
        self.current_servo_angle2 = servo_msg_data.angle2  # 俯仰角

        # self.broadcast_static_gim0_gim2()

    def position_calculate_callback(self, pixel_pos):
        # 计算像素位置相对于相机坐标系的坐标
        pos_x = pixel_pos.pixel_x - PIC_WIDTH / 2
        pos_y = PIC_HEIGHT / 2 - pixel_pos.pixel_y  
        pos_z = -1 * PIC_LONG

        # 创建依赖于 CAM0 的坐标点
        point_source = PointStamped()
        point_source.header.frame_id = "CAM0"
        point_source.header.stamp = pixel_pos.time
        point_source.point.x = pos_x
        point_source.point.y = pos_y
        point_source.point.z = pos_z
        
        # 创建相机原点
        point_origin = PointStamped()
        point_origin.header.frame_id = "CAM0"
        point_origin.header.stamp = pixel_pos.time
        point_origin.point.x = 0
        point_origin.point.y = 0
        point_origin.point.z = 0
        
        try:
            # 转换到世界坐标系
            point_target = self.tf_buffer.transform(point_source, "World", rospy.Duration(1.0))
            point_target0 = self.tf_buffer.transform(point_origin, "World", rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF转换失败: %s", e)
            return
        
        if point_target.point.z < point_target0.point.z:
            # 计算相对坐标
            delta_z = point_target0.point.z - point_target.point.z
            prey_x = point_target.point.x / delta_z * point_target0.point.z
            prey_y = point_target.point.y / delta_z * point_target0.point.z
            # rospy.loginfo("坐标点相对于 world 的坐标:(E: %.6fm N: %.6fm)", prey_x, prey_y)
            
            # 计算经纬度
            obj_lat = self.camera_state.latitude + asin(prey_y / (2 * 1000 * EARTH_RADIUS)) * 2 * 180 / pi
            obj_lon = self.camera_state.longitude + asin(prey_x / (EARTH_RADIUS * 1000 * cos(self.camera_state.latitude * pi / 180) * 2)) * 2 * 180 / pi
            # rospy.loginfo("坐标点相对于 world 的坐标:(E: %.6fm N: %.6fm)", prey_x, prey_y)
            # rospy.loginfo("坐标点经纬度坐标:(E: %.6f N: %.6f)", obj_lon, obj_lat)

            self.save_coordinates_to_json(prey_x, prey_y, obj_lon, obj_lat)
            
            # 填充pos_message
            self.pos_msg.time = pixel_pos.time
            self.pos_msg.relate_E = prey_x
            self.pos_msg.relate_N = prey_y
            self.pos_msg.tar_lat = obj_lat
            self.pos_msg.tar_lon = obj_lon
            
            # 发布位置消息
            self.pub_pos.publish(self.pos_msg)

    def save_coordinates_to_json(self,prey_x, prey_y, obj_lon, obj_lat, file_path='position_test.json'):
        # 获取当前 ROS 时间戳
        timestamp = rospy.Time.now()

        # 生成带时间戳的坐标数据
        coordinates_with_timestamp = {
            'prey_x': prey_x,
            'prey_y': prey_y,
            'obj_lon': obj_lon,
            'obj_lat': obj_lat,
            'timestamp': {
                'secs': timestamp.secs,
                'nsecs': timestamp.nsecs
            }
        }

        # 递加写入到文件
        try:
            with open(file_path, 'a') as f:  # 使用 'a' 模式追加
                json.dump(coordinates_with_timestamp, f, ensure_ascii=False, indent=4)
                f.write("\n")  # 每条记录换行，保持文件格式可读
        except Exception as e:
            rospy.logerr(f"保存坐标数据时出错: {e}")

def main():
    with open('position_test.json', 'w') as f:
        pass
    try:
        node = NPPositionNode()
        # 启动线程发布坐标框架
        thread_static_pub = threading.Thread(target=node.Static_info_pub)
        thread_static_pub.daemon = True  # 设置为守护线程
        thread_static_pub.start()

        # 回调无人机信息
        # relative_altitude_sub = rospy.Subscriber('mavros/global_position/rel_alt', Float64, node.relative_altitude_callback)
        attitude_sub = rospy.Subscriber('mavros/imu/data', Imu, node.attitude_callback)
        # global_position_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, node.global_position_callback)
        global_position_sub = rospy.Subscriber('fake_gps', NavSatFix, node.fakegps_callback)
        # 订阅像素位置主题 /detect
        detect_info_sub = rospy.Subscriber("/detect", pixel_msg, node.position_calculate_callback)
        # 订阅云台角度主题 /servo_angles
        servo_sub = rospy.Subscriber("/servo_angles", servo_msg, node.servo_callback)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
if __name__ == "__main__":
    main()
