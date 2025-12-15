#!/usr/bin/env python3
# -*- coding: UTF-8 -*- 

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from cube_detect_class import PolygonDetector, Cube

class Mycamera:
    def __init__(self):
        self.bridge = CvBridge()
        self.img_received = False
        self.pixel_pos = []  
        self.pic_deep = None  
        
        # 黄色检测参数（HSV空间）[1,2,3](@ref)
        self.lower_yellow = np.array([20, 100, 100])  # H最小值, S最小值, V最小值
        self.upper_yellow = np.array([30, 255, 255])  # H最大值, S最大值, V最大值
        
        # 形态学处理内核[7](@ref)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9,9))
        
        # 初始化订阅器
        self.sub_img = rospy.Subscriber('/camera/color/image_raw', Image, self.detect)
        self.sub_depth = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_image_callback)

        # self.cube_class = 
    def detect(self, msg):
        if not self.img_received:
            self.img_received = True
            rospy.loginfo("Enhanced 3D Cube Detection Initialized...")
        
        try:
            # 图像预处理
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            cube_class = Cube(img, show_flag=True)
            detector = PolygonDetector(min_distance=40, epsilon_ratio=0.02)
            poly_points = detector.detect_polygon(cube_class.canny_image)

            center = detector.get_polygon_center(poly_points)

            # 可视化结果
            vis = img.copy()
            for p in poly_points:
                cv2.circle(vis, tuple(p.astype(int)), 6, (0, 0, 255), -1)
            if center is not None:
                cv2.circle(vis, (int(center[0]), int(center[1])), 8, (0, 255, 0), -1)  # 绿色实心点表示中心
                print("多边形中心点：", center)
                self.pixel_pos.append((int(center[0]), int(center[1])))
            cv2.imshow("Polygon Detection", vis)
            cv2.waitKey(0)
            # # 可视化结果
            # vis = img.copy()
            # for p in poly_points:
            #     cv2.circle(vis, tuple(p.astype(int)), 6, (0, 0, 255), -1)
            # cv2.imshow("Polygon Detection", vis)
            # # 显示优化
            # cv2.imshow("3D Cube Detection", img)
            # cv2.waitKey(1)
            
        except CvBridgeError as e:
            rospy.logerr(f"Vision Processing Error: {str(e)}")

    def depth_image_callback(self, msg):
        if self.img_received and len(self.pixel_pos):
            try:
                depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
                for (x,y) in self.pixel_pos:
                    if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                        self.pic_deep = depth_image[y, x]
                        rospy.loginfo(f"Square Depth at ({x}, {y}): {self.pic_deep:.3f}mm")
                        # 计算相机坐标系下位置
                        
            except CvBridgeError as e:
                rospy.logerr(f"Depth error: {str(e)}")

if __name__ == '__main__':
    rospy.init_node('black_square_detector')
    detector = Mycamera()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()