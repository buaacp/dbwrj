#!/usr/bin/env python3
# -*- coding: UTF-8 -*- 

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

class PositionFilter:
    def __init__(self):
        # 滤波器参数
        self.alpha = rospy.get_param('~alpha', 0.3)  # 平滑因子，可动态配置
        self.min_distance_threshold = rospy.get_param('~min_distance_threshold', 0.05)  # 最小移动距离阈值
        
        # 滤波器状态
        self.filtered_position = np.array([0.0, 0.0, 0.0])  # 默认点 [0, 0, 0]
        self.is_initialized = False
        self.last_valid_position = np.array([0.0, 0.0, 0.0])

        # 这两个矩阵需要在matlab中计算获取
        self.R = np.array([[0.9999, -0.0118, 0.0119],
                        [-0.0109, -0.9975, -0.0698],
                        [0.0127, 0.0696, -0.9975]])
        self.T = np.array([-0.0163,0.1876,0.0349]).reshape(3,1)
        
        # ROS发布器和订阅器 - 修正订阅消息类型
        self.sub_point = rospy.Subscriber('/cube_marker', Marker, self.marker_callback)  # 改为marker_callback
        self.pub_filtered_marker = rospy.Publisher('/filtered_cube_marker', Marker, queue_size=10)
        self.pub_filtered_point = rospy.Publisher('/filtered_cube_position', PointStamped, queue_size=10)
        self.pub_filtered_transed_point = rospy.Publisher('/object_detection/object_position_tripod', PointStamped, queue_size=10)
        self.pub_transformed_marker = rospy.Publisher('/transformed_cube_marker', Marker, queue_size=10)

        rospy.loginfo("Position filter initialized with alpha=%.2f", self.alpha)
    
    def marker_callback(self, msg):  # 重命名回调函数
        """处理接收到的Marker消息并进行滤波"""
        try:
            # 从Marker消息中提取位置信息 - 修正字段访问
            if msg.type == Marker.CUBE or msg.type == Marker.SPHERE:  # 根据实际类型调整
                # 从pose.position获取位置
                current_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            elif msg.type == Marker.POINTS and len(msg.points) > 0:  # 如果是点类型
                # 从points数组获取第一个点
                current_position = np.array([msg.points[0].x, msg.points[0].y, msg.points[0].z])
            else:
                rospy.logwarn("Unsupported marker type: %d", msg.type)
                return
            
            # 检查坐标有效性（排除异常值）
            if self.is_valid_position(current_position):
                # 应用滤波
                filtered_pos = self.apply_filter(current_position)
                
                # 发布滤波后的Marker
                self.publish_filtered_marker(filtered_pos, msg.header)
                
                # 发布滤波后的坐标点（可选）
                self.publish_filtered_point(filtered_pos, msg.header)
            else:
                rospy.logwarn("Invalid position received: [%.3f, %.3f, %.3f]", 
                             current_position[0], current_position[1], current_position[2])
                
        except Exception as e:
            rospy.logerr("Error in marker_callback: %s", str(e))
    
    def is_valid_position(self, position):
        """检查坐标是否有效"""
        # 检查是否为NaN或无穷大
        if np.any(np.isnan(position)) or np.any(np.isinf(position)):
            return False
        
        # # 检查坐标是否在合理范围内（根据你的应用场景调整）
        if np.any(np.abs(position) > 5.0):  # 假设最大范围5米
            return False
            
        # # 如果是第一次接收数据，直接认为是有效的
        if not self.is_initialized:
            return True
            
        # 检查移动距离是否过大（可能是异常跳变）
        distance = np.linalg.norm(position - self.last_valid_position)
        if distance > 0.5:  # 如果单次移动超过2米，认为是异常
            rospy.logwarn("Large position jump detected: %.3f meters", distance)
            return False
            
        return True
    
    def apply_filter(self, current_position):
        """应用指数加权移动平均滤波"""
        if not self.is_initialized:
            # 第一次接收数据，直接使用当前值
            self.filtered_position = current_position
            self.is_initialized = True
        else:
            # 计算与上一次滤波位置的距离
            distance = np.linalg.norm(current_position - self.filtered_position)
            
            # 如果移动距离很小，增加平滑度（减少抖动）
            adaptive_alpha = self.alpha
            if distance < self.min_distance_threshold:
                adaptive_alpha = self.alpha * 0.5  # 小距离移动时更平滑
            
            # 应用EWMA滤波
            self.filtered_position = (adaptive_alpha * current_position + 
                                    (1 - adaptive_alpha) * self.filtered_position)
        
        self.last_valid_position = current_position
        return self.filtered_position
    
    def publish_filtered_marker(self, position, header):
        """发布滤波后的Marker"""
        marker = Marker()
        
        # 设置Marker的基本属性[6,8](@ref)
        marker.header.frame_id = header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "filtered_cube"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD  # 添加或修改操作[3](@ref)
        
        # 设置位置和姿态
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # 设置尺寸
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # 设置颜色（确保alpha值不为0）[6,8](@ref)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0  # 蓝色
        marker.color.a = 0.8  # 必须设置透明度不为0
        
        # 设置Marker的生存时间
        marker.lifetime = rospy.Duration(0.2) 
        
        self.pub_filtered_marker.publish(marker)
    
    def publish_filtered_point(self, position, header):
            """发布滤波后的坐标点"""

            rospy.loginfo("=== 坐标变换调试信息 ===")
            rospy.loginfo("原始坐标 (相机坐标系): [%.6f, %.6f, %.6f]", 
                 position[0], position[1], position[2])
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
            
            transed_point_msg = PointStamped()
            transed_point_msg.header.frame_id = header.frame_id
            transed_point_msg.header.stamp = rospy.Time.now()
            transed_point_msg.point.x = transed_position[0, 0]  # 注意索引，因为现在是(3,1)矩阵
            transed_point_msg.point.y = transed_position[1, 0]
            transed_point_msg.point.z = transed_position[2, 0]
        
            self.pub_filtered_transed_point.publish(transed_point_msg)

            rospy.loginfo("最终变换坐标 (R*P + T): [%.6f, %.6f, %.6f]", 
                transed_position[0,0], transed_position[1,0], transed_position[2,0])
            # 发布转换点的可视化Marker
            self.publish_transformed_marker(transed_position, header)
    
    def publish_transformed_marker(self, transed_position, header):
        """发布转换后的点可视化Marker"""
        marker = Marker()
        
        # 设置Marker的基本属性[7](@ref)
        marker.header.frame_id = header.frame_id  # 使用目标坐标系
        marker.header.stamp = rospy.Time.now()
        marker.ns = "transformed_cube"
        marker.id = 1  # 使用不同的ID以区分原始滤波点
        marker.type = Marker.SPHERE  # 使用球体表示转换后的点[7](@ref)
        marker.action = Marker.ADD
        
        # 设置位置[4](@ref)
        marker.pose.position.x = transed_position[0, 0]
        marker.pose.position.y = transed_position[1, 0]
        marker.pose.position.z = transed_position[2, 0]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # 设置尺寸 - 比原始点大一些以便区分
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        
        # 设置颜色 - 使用不同颜色区分（例如红色）[4](@ref)
        marker.color.r = 1.0  # 红色
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.9  # 透明度
        
        # 设置生存时间
        marker.lifetime = rospy.Duration(0.5)  # 0.5秒后自动消失，显示动态效果
        
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