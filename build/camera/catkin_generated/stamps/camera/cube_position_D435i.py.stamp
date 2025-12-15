#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*- 

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
import sys
import os

# 添加当前脚本所在目录到sys.path
current_dir = os.path.dirname(__file__)
sys.path.append(current_dir)
from cube_detect_class import PolygonDetector, Cube

class Mycamera:
    def __init__(self):
        self.bridge = CvBridge()
        self.img_received = False
        self.pixel_pos = []  
        self.pic_deep = None  
        self.intrinsic_ready = False

        # ===== 自动更新的相机内参 =====
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        
        # ===== ROS 订阅与发布 =====
        self.sub_caminfo = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.cam_info_callback)
        self.sub_img = rospy.Subscriber('/camera/color/image_raw', Image, self.detect)
        self.sub_depth = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_image_callback)
        self.pub_marker = rospy.Publisher('/cube_marker', Marker, queue_size=10)

        # TF 缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def cam_info_callback(self, msg):
        """自动从 camera_info 获取内参"""
        if not self.intrinsic_ready:
            self.fx = msg.K[0]
            self.fy = msg.K[4]
            self.cx = msg.K[2]
            self.cy = msg.K[5]
            self.intrinsic_ready = True
            rospy.loginfo("Camera intrinsics loaded: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}".format(
                fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy))
            self.sub_caminfo.unregister()  # 加载一次即可

    def detect(self, msg):
        if not self.img_received:
            self.img_received = True
            rospy.loginfo("Enhanced 3D Cube Detection Initialized...")
        
        try:
            # 图像预处理
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            self.pixel_pos = []

            cube_class = Cube(img, show_flag=False)
            detector = PolygonDetector(min_distance=40, epsilon_ratio=0.02)
            poly_points = detector.detect_polygon(cube_class.canny_image)

            center = detector.get_polygon_center(poly_points)

            # 可视化结果
            vis = img.copy()
            for p in poly_points:
                cv2.circle(vis, tuple(p.astype(int)), 6, (0, 0, 255), -1)
            if center is not None:
                cv2.circle(vis, (int(center[0]), int(center[1])), 8, (0, 255, 0), -1)  # 绿色实心点表示中心
                # print("多边形中心点：", center)
                self.pixel_pos.append((int(center[0]), int(center[1])))
            cv2.imshow("Polygon Detection", vis)
            cv2.waitKey(1)
            
        except CvBridgeError as e:
            rospy.logerr("Vision Processing Error: {}".format(str(e)))

    def depth_image_callback(self, msg):
        """深度图回调 + 坐标计算 + TF 转换 + 发布"""
        if not (self.img_received and self.intrinsic_ready and len(self.pixel_pos)):
            return
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            for (x, y) in self.pixel_pos:
                if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                    depth_value = depth_image[y, x]
                    if np.isnan(depth_value) or depth_value <= 0:
                        continue
                    
                    Z = depth_value
                    X = (x - self.cx) * Z / self.fx
                    Y = (y - self.cy) * Z / self.fy

                    # 构造相机坐标下点
                    marker = Marker()
                    marker.header.frame_id = "camera_color_optical_frame"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "cube"
                    marker.id = 0
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = X/1000
                    marker.pose.position.y = Y/1000
                    marker.pose.position.z = Z/1000
                    marker.scale.x = 0.05
                    marker.scale.y = 0.05
                    marker.scale.z = 0.05
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.3
                    self.pub_marker.publish(marker)

                    rospy.loginfo("[Camera] X:{X:.3f} Y:{Y:.3f} Z:{Z:.3f}".format(X=X, Y=Y, Z=Z))
                    # X_UAV = X/10
                    # Y_UAV = -1*Y/10
                    # Z_UAV = -1*Z/10
                    # rospy.loginfo("[无人机坐标系下的位置] X:{X:.3f} Y:{Y:.3f} Z:{Z:.3f}".format(
                    #     X=X_UAV, Y=Y_UAV, Z=Z_UAV))

        except CvBridgeError as e:
            rospy.logerr("Depth error: {}".format(str(e)))

if __name__ == '__main__':
    rospy.init_node('yellow_cube_locator')
    detector = Mycamera()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()