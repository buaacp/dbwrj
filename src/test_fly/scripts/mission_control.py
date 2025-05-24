#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
import sys
from std_msgs.msg import Int32

class MissionPublisher:
    def __init__(self):
        rospy.init_node('mission_publisher', anonymous=True)
        self.pub = rospy.Publisher('/mission_state', Int32, queue_size=10)
        rospy.sleep(0.5)  # 等待发布者注册

    def publish_value(self, value):
        try:
            msg = Int32(data=int(value))
            self.pub.publish(msg)
            rospy.loginfo("Published: %d", msg.data)
        except ValueError:
            rospy.logerr("Invalid input: %s (must be integer)", value)

def main():
    pub_node = MissionPublisher()
    
    # 模式1：命令行参数发布（支持多参数）
    if len(sys.argv) > 1:
        for arg in sys.argv[1:]:
            pub_node.publish_value(arg)
        return
    
    # 模式2：交互式发布
    rospy.loginfo("Enter mission states (Ctrl+C to exit):")
    while not rospy.is_shutdown():
        try:
            user_input = input(">> ").strip()
            if user_input.lower() in ['exit', 'quit']:
                break
            pub_node.publish_value(user_input)
        except KeyboardInterrupt:
            rospy.signal_shutdown("User exit")
        except Exception as e:
            rospy.logerr("Input error: %s", str(e))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass