#!/usr/bin/env python
# -*- coding: UTF-8 -*- 
import rospy
import numpy as np
import tf2_ros
import geometry_msgs.msg
from scipy.spatial.transform import Rotation as R

def quaternion_to_rotation_matrix(q):
    """将四元数转换为3x3旋转矩阵 (四元数顺序为x,y,z,w) [1,3](@ref)"""
    return R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

def build_transform_matrix(translation, rotation):
    """构建4x4齐次变换矩阵 [1,3,6](@ref)"""
    # 创建4x4单位矩阵
    T = np.eye(4)
    
    # 设置旋转分量
    T[:3, :3] = quaternion_to_rotation_matrix(rotation)
    
    # 设置平移分量
    T[:3, 3] = [translation.x, translation.y, translation.z]
    
    return T

def main():
    rospy.init_node('gimbal_tf_listener')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    while not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform("GIM0", "GIM2", rospy.Time(0), rospy.Duration(1.0))
            
            # 提取平移和旋转数据
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            
            # 构建变换矩阵
            T = build_transform_matrix(translation, rotation)
            
            # 格式化打印矩阵
            np.set_printoptions(precision=4, suppress=True)
            rospy.loginfo("\n4x4 Transform Matrix:\n" + str(T))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn(f"暂时无法获取变换: {e}, 等待重试...")
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr(f"时间外推错误: {e}")
            break
        
        rospy.sleep(1)

if __name__ == "__main__":
    main()