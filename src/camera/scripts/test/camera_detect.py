#! /usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import numpy as np
import cv2 
from math import cos, sin, tan, sqrt, pi

import threading

from std_msgs.msg import Float64
from camera.msg import pixel_msg

pic_width = 640
pic_height = 480
pic_long = 539

#obj_squre = 0.05
obj_squre = 0.5



# 修改函数名，避免与全局变量冲突
def relative_altitude_callback(msg):
    global relative_altitude
    relative_altitude = msg.data

# 修改函数名，避免与全局变量冲突
def relative_altitude_info_update():
    sub_Attitude = rospy.Subscriber('mavros/global_position/rel_alt', Float64, relative_altitude_callback)
    rospy.spin()

# 修改函数名，添加返回值
def detect_white_ball_from_camera(frame, min_Radius, max_Radius):
    # 转换图像为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    v_up = 40
    v_down = 256

    h_up = 160
    h_down =256

    # 定义红色的HSV范围
    lower_red1 = np.array([0, h_up, v_up])
    upper_red1 = np.array([4, h_down, v_down])

    lower_red2 = np.array([177, h_up, v_up])
    upper_red2 = np.array([180, h_down, v_down])

    lower_light = np.array([0,0,253])
    upper_light = np.array([180,255,255])


    # 定义反光
    mask_light = cv2.inRange(hsv,lower_light,upper_light)


    # 根据HSV范围创建掩膜
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red  = cv2.bitwise_or(mask_red1, mask_red2)
    mask = cv2.bitwise_or(mask_red, mask_light)
    mask = cv2.erode(mask, None, iterations=3)  # erossion
    mask = cv2.dilate(mask, None, iterations=3)






    # 在原图上应用掩膜
    result = cv2.bitwise_and(frame, frame, mask=mask)


    # 转换为灰度图像
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    # 将像素值大于1的像素设置为255
    threshold_value = 1
    gray[gray > threshold_value] = 255
    # 显示结果
    cv2.imshow('gray', gray)

    # 进行圆检测
    # param1 用于检测边缘。这个值越高，检测到的边缘越强
    # param2 阈值越低，检测到的圆越多。这里设置为30
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=40,
        param1=50,
        param2=8,
        minRadius=min_Radius,
        maxRadius=max_Radius
    )

    # 如果找到圆，绘制圆
    if circles is not None:
        circles = np.uint16(np.around(circles))
        cnt = 0
        for i in circles[0, :]:
            if cnt==0:
                cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                break
        cv2.imshow('White Ball Detection', frame)
        return 1, i[0], i[1]  # 返回1和圆心坐标
    else:
        cv2.imshow('White Ball Detection', frame)
        return 0, 0, 0  # 返回0和默认坐标

if __name__ == "__main__":
    global relative_altitude
    relative_altitude = 0.5
    rospy.init_node('np_Camera', anonymous=True)
    pub_pixel = rospy.Publisher("/detect", pixel_msg, queue_size=10)

    # 启动线程回调高度
    thread_Sub = threading.Thread(target=relative_altitude_info_update)
    thread_Sub.start()

    rate = rospy.Rate(10)

    # 创建一个空的pixel_msg
    msg = pixel_msg()

    # 打开摄像头
    cap = cv2.VideoCapture(0)  # 0表示默认的摄像头

    while not rospy.is_shutdown():
        # 读取帧
        ret, frame = cap.read()
        print(relative_altitude)

        # 调用函数
        if ret and relative_altitude>2:
            time_record = rospy.Time.now()
            radiu_expect = sqrt(obj_squre / pi) * pic_long / relative_altitude
            #print("期望半径（像素）：%d 飞行高度 ：%f" %(radiu_expect,relative_altitude))
            min_Radius = int(0.6 * radiu_expect-1)
            max_Radius = int(1.5 * radiu_expect+1)
            # print("最小半径: %d  最大半径: %d" % (min_Radius, max_Radius))  # 修改打印语句
            flag_detect, x, y = detect_white_ball_from_camera(frame, min_Radius, max_Radius)
            if flag_detect:
                msg.pixel_x = x
                msg.pixel_y = y
                msg.time = time_record
                print("像素x: %d  像素y: %d" % (x, y))  # 修改打印语句
                pub_pixel.publish(msg)

        rate.sleep()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
