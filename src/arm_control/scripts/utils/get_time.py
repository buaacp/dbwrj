import rospy
from rosgraph_msgs.msg import Clock

def clock_callback(msg):
    # 从消息中提取仿真时间（单位：秒）
    sim_time = msg.clock.to_sec()
    rospy.loginfo(f"Current Gazebo time: {sim_time:.2f} seconds")

if __name__ == "__main__":
    # 初始化节点，启用仿真时间模式
    rospy.init_node("gazebo_clock_subscriber")
    # 订阅 /clock 话题
    rospy.Subscriber("/clock", Clock, clock_callback)
    rospy.spin()  # 保持节点运行