#!/usr/bin/env python3
# -*- coding: UTF-8 -*- 

import rospy
import numpy as np
import tf
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

class PositionFilter:
    def __init__(self):
        # 滤波器参数
        self.alpha = rospy.get_param('~alpha', 0.3)
        self.min_distance_threshold = rospy.get_param('~min_distance_threshold', 0.05)
        
        # 滤波器状态
        self.filtered_position = np.array([0.0, 0.0, 0.0])
        self.is_initialized = False
        self.last_valid_position = np.array([0.0, 0.0, 0.0])

        # 坐标变换矩阵
        self.R = np.array([[0.9999, -0.0118, 0.0119],
                        [-0.0109, -0.9975, -0.0698],
                        [0.0127, 0.0696, -0.9975]])
        self.T = np.array([-0.0163, 0.1876, 0.0349]).reshape(3,1)
        
        # 独立坐标系参数
        self.arm_base_position = np.array([0.0, 0.0, 0.0])  # arm_base_link的初始位置
        self.arm_base_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # 单位四元数
        
        # ROS发布器和订阅器
        self.sub_point = rospy.Subscriber('/cube_marker', Marker, self.marker_callback)
        self.pub_filtered_marker = rospy.Publisher('/filtered_cube_marker', Marker, queue_size=10)
        self.pub_filtered_point = rospy.Publisher('/filtered_cube_position', PointStamped, queue_size=10)
        self.pub_filtered_transed_point = rospy.Publisher('/object_detection/object_position_tripod', PointStamped, queue_size=10)
        self.pub_transformed_marker = rospy.Publisher('/transformed_cube_marker', Marker, queue_size=10)
        
        # TF坐标变换广播器
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        rospy.loginfo("Position filter initialized with alpha=%.2f", self.alpha)
        rospy.loginfo("arm_base_link will be published as independent frame")
    
    def marker_callback(self, msg):
        """处理接收到的Marker消息并进行滤波"""
        try:
            # 从Marker消息中提取位置信息
            if msg.type == Marker.CUBE or msg.type == Marker.SPHERE:
                current_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            elif msg.type == Marker.POINTS and len(msg.points) > 0:
                current_position = np.array([msg.points[0].x, msg.points[0].y, msg.points[0].z])
            else:
                rospy.logwarn("Unsupported marker type: %d", msg.type)
                return
            
            rospy.logdebug("Callback received: frame_id='%s', position=[%.3f, %.3f, %.3f]", 
                          msg.header.frame_id, current_position[0], current_position[1], current_position[2])
            
            if self.is_valid_position(current_position):
                filtered_pos = self.apply_filter(current_position)
                
                self.publish_filtered_marker(filtered_pos, msg.header)
                self.publish_filtered_point(filtered_pos, msg.header)
            else:
                rospy.logwarn("Invalid position received: [%.3f, %.3f, %.3f]", 
                             current_position[0], current_position[1], current_position[2])
                
        except Exception as e:
            rospy.logerr("Error in marker_callback: %s", str(e))
    
    def is_valid_position(self, position):
        """检查坐标是否有效"""
        if np.any(np.isnan(position)) or np.any(np.isinf(position)):
            return False
        if np.any(np.abs(position) > 5.0):
            return False
        if not self.is_initialized:
            return True
        distance = np.linalg.norm(position - self.last_valid_position)
        if distance > 0.5:
            rospy.logwarn("Large position jump detected: %.3f meters", distance)
            return False
        return True
    
    def apply_filter(self, current_position):
        """应用指数加权移动平均滤波"""
        if not self.is_initialized:
            self.filtered_position = current_position
            self.is_initialized = True
        else:
            distance = np.linalg.norm(current_position - self.filtered_position)
            adaptive_alpha = self.alpha
            if distance < self.min_distance_threshold:
                adaptive_alpha = self.alpha * 0.5
            
            self.filtered_position = (adaptive_alpha * current_position + 
                                    (1 - adaptive_alpha) * self.filtered_position)
        
        self.last_valid_position = current_position
        return self.filtered_position
    
    def publish_filtered_marker(self, position, header):
        """发布滤波后的Marker"""
        marker = Marker()
        marker.header.frame_id = header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "filtered_cube"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8
        
        marker.lifetime = rospy.Duration(0.2)
        
        self.pub_filtered_marker.publish(marker)
    
    def publish_filtered_point(self, position, header):
        """发布滤波后的坐标点"""
        point_msg = PointStamped()
        point_msg.header.frame_id = header.frame_id
        point_msg.header.stamp = rospy.Time.now()
        point_msg.point.x = position[0]
        point_msg.point.y = position[1]
        point_msg.point.z = position[2]
        self.pub_filtered_point.publish(point_msg)

        # 应用坐标变换
        position_vec = np.array([position[0], position[1], position[2]]).reshape(3,1)
        transed_position = np.dot(self.R, position_vec) + self.T
        
        
        # 发布转换后的坐标点
        transed_point_msg = PointStamped()
        transed_point_msg.header.frame_id = "arm_base_link"
        transed_point_msg.header.stamp = rospy.Time.now()
        transed_point_msg.point.x = transed_position[0, 0]
        transed_point_msg.point.y = transed_position[1, 0]
        transed_point_msg.point.z = transed_position[2, 0]
        self.pub_filtered_transed_point.publish(transed_point_msg)
        
        # 发布独立的arm_base_link坐标系
        self.publish_independent_arm_base_link()
        
        # 发布转换点的可视化Marker
        self.publish_transformed_marker(transed_position, header)
    
    def publish_independent_arm_base_link(self):
        """发布独立的arm_base_link坐标系"""
        try:
            current_time = rospy.Time.now()
            
            # 方法1: 将arm_base_link作为map坐标系的子坐标系（固定位置）
            # 这里arm_base_link的位置是固定的，不随检测目标移动
            translation = (self.arm_base_position[0], 
                         self.arm_base_position[1], 
                         self.arm_base_position[2])
            
            rotation = (self.arm_base_orientation[0],  # x
                       self.arm_base_orientation[1],  # y  
                       self.arm_base_orientation[2],  # z
                       self.arm_base_orientation[3])  # w
            
            # 发布到map坐标系（或其他固定坐标系）
            self.tf_broadcaster.sendTransform(
                translation,
                rotation,
                current_time,
                "arm_base_link",  # 子坐标系
                "map"             # 父坐标系（使用map作为参考系）
            )
            
            rospy.logdebug("Published independent arm_base_link frame")
            
        except Exception as e:
            rospy.logwarn("Error publishing independent arm_base_link TF: %s", str(e))

    
    def publish_transformed_marker(self, transed_position, header):
        """发布转换后的点可视化Marker"""
        marker = Marker()
        marker.header.frame_id = "arm_base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "transformed_cube"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = transed_position[0, 0]
        marker.pose.position.y = transed_position[1, 0]
        marker.pose.position.z = transed_position[2, 0]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        
        marker.lifetime = rospy.Duration(0.5)
        
        self.pub_transformed_marker.publish(marker)

def main():
    rospy.init_node('position_filter')
    try:
        filter_node = PositionFilter()
        rospy.loginfo("Position filter node started successfully")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Position filter node shutdown")

if __name__ == '__main__':
    main()