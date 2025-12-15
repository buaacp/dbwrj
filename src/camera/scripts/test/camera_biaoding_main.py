import math
import numpy as np  
import rospy
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
import threading
from sensor_msgs.msg import Image
import tf2_ros
import geometry_msgs.msg
from scipy.spatial.transform import Rotation as R
state_end = []

class PIC_SAVE:
    def __init__(self):
        self.pic_num = 0
        self.rgb_save_trigger = False
    def img_callback(self,msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.rgb_save_trigger:
            rgb_name = "/home/zhenglihaoqi/robotic_arm_ws/src/camera/scripts/test/img/"+str(self.pic_num)+".jpg"
            cv2.imwrite(rgb_name, cv_image)
            print("rgb saved")
            self.pic_num+=1
            self.rgb_save_trigger = False


def M2state(M):
    """
    返回齐次矩阵对应位姿（rad）
    """
    theta_z = math.atan2(-M[2][0], M[0][0])
    theta_y = math.atan2(-M[2][0], math.sqrt(M[2][0]**2 + M[2][2]**2))
    theta_x = math.atan2(-M[2][1], M[2][2])
    pos = [M[0][0], M[0][1], M[0][2]]
    x_state = pos + [theta_x, theta_y, theta_z]
    return x_state

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

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('camera_node')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Rate(50)
    pic_save = PIC_SAVE()
    sub_img = rospy.Subscriber('/d435/color/image_raw', Image, pic_save.img_callback)
    time.sleep(0.1)
    # 启动一个单独的线程来运行 rospy.spin()
    spin_thread = threading.Thread(target=rospy.spin)
    spin_thread.start()
    time.sleep(0.1)
    while not rospy.is_shutdown():
        user_input = input("Press Y to save pose / S to exit: ")
        if user_input == "Y":
            pic_save.rgb_save_trigger = True
            try:
                transform = tf_buffer.lookup_transform("GIM0", "GIM2", rospy.Time(0), rospy.Duration(1.0))
                
                # 提取平移和旋转数据
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                
                # 构建变换矩阵
                T = build_transform_matrix(translation, rotation)
                state_end.append(T)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
                rospy.logwarn(f"暂时无法获取变换: {e}, 等待重试...")
            except tf2_ros.ExtrapolationException as e:
                rospy.logerr(f"时间外推错误: {e}")
                break
            
        elif user_input == "S":
            m = np.array(state_end)
            np.save("/home/zhenglihaoqi/robotic_arm_ws/src/camera/scripts/test/pose/state.npy", m)
            print("saved")
            break

