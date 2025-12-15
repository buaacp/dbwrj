#!/usr/bin/env python2
# -*- coding: UTF-8 -*-
# 1.导包
import rospy
from camera.msg import pos_message,detect_msg
import math
import time
import json

end_distance=10 #融合距离 m
end_conf = 8
PI=3.1415926535897932384626433832795
EARTH_RADIUS=6378.137 #地球半径 KM
y=float()
x=float()
global if_target_selected 
if_target_selected= False
global target
target = []

# 创建一个列表，用于存储目标点的坐标、置信度、是否发送 未发送为0
target_points = []


def save_target_points_with_timestamp(target_points, filename="target_points.json"):
    # 获取当前 ROS 时间戳
    timestamp = rospy.Time.now()
    
    # 生成带时间戳的目标点数据
    target_data_with_timestamp = []
    for point in target_points:
        target_data_with_timestamp.append({
            'lat': point[0],
            'lon': point[1],
            'confidence': point[2],
            'timestamp': {
                'secs': timestamp.secs,
                'nsecs': timestamp.nsecs
            }
        })
    if(if_target_selected):
        target_data_with_timestamp.append({
                'lat': target[0],
                'lon': target[1],
                'isTarget': True,
                'confidence': target[2],
                'timestamp': {
                    'secs': timestamp.secs,
                    'nsecs': timestamp.nsecs
                }
            })
    
    # 递加写入到文件
    try:
        with open(filename, 'a') as f:  # 使用 'a' 模式追加
            for data in target_data_with_timestamp:
                json.dump(data, f)
                f.write("\n")  # 每条记录换行，保持文件格式可读
            f.write("\n\n") 
        # print(f"Target points with timestamp saved to {filename}")
    except Exception as e:
        print(f"Error saving target points: {e}")

def distance_calculate(lat1,lon1,lat2,lon2):
    radlat=abs(lat1-lat2)*PI/180.0 #相对角度换算成弧度
    y = 2*math.sin(radlat/2)*EARTH_RADIUS*1000 #输出单位m
    radlon=abs(lon1-lon2)*PI/180.0	#相对角度换算成弧度
    radlat=lat1*PI/180
    x=2*math.sin(radlon/2)*math.cos(radlat)*EARTH_RADIUS*1000	#输出单位m
    distance = float(math.sqrt(x**2+y**2))
    return distance

def domessage(message):
    global if_target_selected,target,last_pub_time
    if(if_target_selected):
        if(distance_calculate(message.tar_lat,message.tar_lon,target[0],target[1]))<=5:
            target[2]*=0.9
            target[0]=(target[0]*target[2]+message.tar_lat)/(target[2]+1)
            target[1]=(target[1]*target[2]+message.tar_lon)/(target[2]+1)
            target[2]+=1
        msg = detect_msg()
        msg.lat=target[0]
        msg.lon=target[1]
        now_time = time.time()
        if(now_time-last_pub_time>=1):
            last_pub_time = now_time
            pub.publish(msg)
    else:
        if target_points :
            min_distance=1000
            for point in target_points:
                distance = distance_calculate(message.tar_lat,message.tar_lon,point[0],point[1])
                if distance < min_distance:
                    min_distance = distance
                    min_index = target_points.index(point)
            if min_distance>end_distance:
                #写入新的点
                target_points.append([message.tar_lat,message.tar_lon,1.0])
            else:
                #融合
                # print("fusing")
                lat_end=(target_points[min_index][0]*target_points[min_index][2]+message.tar_lat)/(target_points[min_index][2]+1)
                lon_end=(target_points[min_index][1]*target_points[min_index][2]+message.tar_lon)/(target_points[min_index][2]+1)
                target_points[min_index]=(lat_end,lon_end,target_points[min_index][2]+1)

        else:
            target_points.append([message.tar_lat,message.tar_lon,1.0])

        #发布消息
        for point in target_points:
            if point[2]>end_conf:
                if_target_selected = True
                target=[point[0],point[1],point[2]+0.0]
                msg = detect_msg()
                msg.lat=point[0]
                msg.lon=point[1]
                pub.publish(msg)




if __name__ == "__main__":
    with open('target_points.json', 'w') as f:
        pass
    # 2.初始化 ROS 节点
    rospy.init_node("pos_analyze")
    pub = rospy.Publisher('/location_topic', detect_msg, queue_size=10)
    sub_pos = rospy.Subscriber("/pos_message", pos_message, domessage)
    rate = rospy.Rate(1)  
    global last_pub_time
    last_pub_time = time.time()
    while not rospy.is_shutdown():
        for i in range(len(target_points)):
                # print('decending')
                target_points[i]=(target_points[i][0],target_points[i][1],target_points[i][2]-1)
        # 这里保存
        if target_points:
            save_target_points_with_timestamp(target_points)  # 保存目标点数据
        rate.sleep()
