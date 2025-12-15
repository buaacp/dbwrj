#!/usr/bin/env python
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

        # 黄色检测参数（HSV空间）
        # self.lower_yellow = np.array([20, 100, 100])
        # self.upper_yellow = np.array([30, 255, 255])
        self.lower_yellow = np.array([15, 100, 135])
        self.upper_yellow = np.array([40, 255, 255])
        
        # 形态学内核
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9,9))

        # ===== ROS 订阅与发布 =====
        self.sub_caminfo = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.cam_info_callback)
        self.sub_img = rospy.Subscriber('/camera/color/image_raw', Image, self.detect)
        self.sub_depth = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_image_callback)
        self.pub_marker = rospy.Publisher('/cube_marker', Marker, queue_size=10)
        # 发布三维点（世界坐标）
        self.pub_point = rospy.Publisher('/cube_position', PointStamped, queue_size=10)

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
            rospy.loginfo(f"Camera intrinsics loaded: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")
            self.sub_caminfo.unregister()  # 加载一次即可

    def detect(self, msg):
        if not self.img_received:
            self.img_received = True
            rospy.loginfo("Enhanced 3D Cube Detection Initialized...")
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            mask = cv2.inRange(hsv_image, self.lower_yellow, self.upper_yellow)
            hex_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, hex_kernel, iterations=4)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, hex_kernel, iterations=2)
            
            contour_info = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours = contour_info[0] if len(contour_info) == 2 else contour_info[1]
            hierarchy = contour_info[1] if len(contour_info) == 2 else contour_info[2]
            self.pixel_pos = []
            
            for i, cnt in enumerate(contours):
                if hierarchy[0][i][3] != -1:
                    continue
                if cv2.contourArea(cnt) < 800:
                    continue
                    
                epsilon = 0.015 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                if len(approx) not in range(4,9):
                    continue
                    
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w)/h
                area_ratio = cv2.contourArea(cnt)/(w*h)
                
                angles = []
                for j in range(len(approx)):
                    pt1 = approx[j-2][0] if j>=2 else approx[len(approx)+j-2][0]
                    pt2 = approx[j-1][0]
                    pt3 = approx[j][0]
                    vec1 = pt1 - pt2
                    vec2 = pt3 - pt2
                    angle = np.arccos(np.dot(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2)))
                    angles.append(np.degrees(angle))
                
                valid_angles = sum(1 for a in angles if 110 < a < 130 or 50 < a < 70) >= 3
                valid_aspect = 0.5 < aspect_ratio < 2.0
                valid_area = 0.4 < area_ratio < 0.9
                
                if valid_angles and valid_aspect and valid_area:
                    M = cv2.moments(cnt)
                    if M['m00'] != 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        cv2.drawContours(cv_image, [approx], -1, (0,255,0), 3)
                        cv2.circle(cv_image, (cx,cy), 5, (0,0,255), -1)
                        cv2.putText(cv_image, "Cube", (cx-30, cy-20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
                        self.pixel_pos.append((cx, cy))

            cv2.imshow("3D Cube Detection", cv_image)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)
            
        except CvBridgeError as e:
            rospy.logerr(f"Vision Processing Error: {str(e)}")

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
                    marker.type = Marker.SPHERE
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
                    marker.color.a = 1.0
                    self.pub_marker.publish(marker)
                    # point_cam = PointStamped()
                    # point_cam.header.stamp = rospy.Time.now()
                    # point_cam.header.frame_id = "camera_color_optical_frame"
                    # point_cam.point.x = X
                    # point_cam.point.y = Y
                    # point_cam.point.z = Z

                    # self.pub_point.publish(point_cam)
                    rospy.loginfo(f"[Camera] X:{X:.3f} Y:{Y:.3f} Z:{Z:.3f}")

        except CvBridgeError as e:
            rospy.logerr(f"Depth error: {str(e)}")

if __name__ == '__main__':
    rospy.init_node('yellow_cube_locator')
    detector = Mycamera()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
