#!/usr/bin/env python
# -*- coding: UTF-8 -*- 

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

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

    def detect(self, msg):
        if not self.img_received:
            self.img_received = True
            rospy.loginfo("Enhanced 3D Cube Detection Initialized...")
        
        try:
            # 图像预处理
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # 创建颜色掩膜（保持原有参数）
            mask = cv2.inRange(hsv_image, self.lower_yellow, self.upper_yellow)
            
            # 形态学处理优化（网页7建议）
            hex_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11))  # 增大内核尺寸
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, hex_kernel, iterations=4)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, hex_kernel, iterations=2)
            
            # 轮廓检测（兼容OpenCV版本）
            contour_info = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours = contour_info[0] if len(contour_info) == 2 else contour_info[1]
            hierarchy = contour_info[1] if len(contour_info) == 2 else contour_info[2]
            self.pixel_pos = []
            
            for i, cnt in enumerate(contours):
                # 层级过滤（网页7建议）
                if hierarchy[0][i][3] != -1:
                    continue
                    
                # 面积过滤（网页6建议）
                if cv2.contourArea(cnt) < 800:  # 适当降低面积阈值
                    continue
                    
                # 多边形近似（网页6/7改进）
                epsilon = 0.015 * cv2.arcLength(cnt, True)  # 更精确的拟合
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                
                # 顶点数动态验证（网页8启发）
                if len(approx) not in range(4,9):  # 允许顶点数4-8
                    continue
                    
                # 几何特征验证（网页6/7/8结合）
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w)/h
                area_ratio = cv2.contourArea(cnt)/(w*h)
                
                # 角度验证（网页8核心改进）
                angles = []
                for j in range(len(approx)):
                    pt1 = approx[j-2][0] if j>=2 else approx[len(approx)+j-2][0]
                    pt2 = approx[j-1][0]
                    pt3 = approx[j][0]
                    vec1 = pt1 - pt2
                    vec2 = pt3 - pt2
                    angle = np.arccos(np.dot(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2)))
                    angles.append(np.degrees(angle))
                
                # 判断立方体特征（网页6/8结合）
                valid_angles = sum(1 for a in angles if 110 < a < 130 or 50 < a < 70) >= 3  # 至少3个有效角度
                valid_aspect = 0.5 < aspect_ratio < 2.0  # 放宽宽高比限制
                valid_area = 0.4 < area_ratio < 0.9  # 优化面积比
                
                if valid_angles and valid_aspect and valid_area:
                    # 直线边缘验证（网页6补充）
                    edge_mask = np.zeros_like(mask)
                    cv2.drawContours(edge_mask, [cnt], -1, 255, 1)
                    edges = cv2.Canny(edge_mask, 50, 150)
                    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30, minLineLength=20, maxLineGap=10)
                    
                    if lines is not None and len(lines) >= (6 if len(approx)==6 else 4):
                        # 计算精确中心（网页8方法）
                        M = cv2.moments(cnt)
                        if M['m00'] != 0:
                            cx = int(M['m10']/M['m00'])
                            cy = int(M['m01']/M['m00'])
                            
                            # 绘制动态标记
                            color = (0,255,0) if len(approx)==4 else (255,0,0)
                            cv2.drawContours(cv_image, [approx], -1, color, 3)
                            cv2.circle(cv_image, (cx,cy), 5, (0,0,255), -1)
                            
                            # 添加形状标注
                            label = f"Cube({len(approx)}-sides)"
                            cv2.putText(cv_image, label, (cx-40, cy-30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                            self.pixel_pos.append((cx, cy))

            # 显示优化
            cv2.imshow("3D Cube Detection", cv_image)
            cv2.imshow("Morphological Mask", mask)
            cv2.waitKey(1)
            
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