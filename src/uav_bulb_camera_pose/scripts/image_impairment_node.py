#!/usr/bin/env python3
import random

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

from uav_bulb_camera_pose.ros_image_numpy import bgr8_to_imgmsg, imgmsg_to_bgr8


class ImageImpairmentNode(object):
    def __init__(self):
        self.noise_stddev = float(rospy.get_param('~image_noise_stddev', 0.0))
        self.blur_kernel = int(rospy.get_param('~motion_blur_kernel', 0))
        self.brightness_gain = float(rospy.get_param('~brightness_gain', 1.0))
        self.contrast_gain = float(rospy.get_param('~contrast_gain', 1.0))
        self.frame_delay_ms = int(rospy.get_param('~frame_delay_ms', 0))
        self.drop_probability = float(rospy.get_param('~frame_drop_probability', 0.0))
        self.jpeg_quality = int(rospy.get_param('~jpeg_quality', 100))
        in_topic = rospy.get_param('~input_image_topic', '/camera/camera/color/image_raw_clean')
        out_topic = rospy.get_param('~output_image_topic', '/camera/camera/color/image_raw')
        self.pub = rospy.Publisher(out_topic, Image, queue_size=3)
        self.sub = rospy.Subscriber(in_topic, Image, self.cb, queue_size=3, buff_size=2 ** 24)

    def cb(self, msg):
        if random.random() < self.drop_probability:
            return
        image = imgmsg_to_bgr8(msg)
        out = image.astype(np.float32)
        out = (out - 127.5) * self.contrast_gain + 127.5
        out = out * self.brightness_gain
        if self.noise_stddev > 0.0:
            out += np.random.normal(0.0, self.noise_stddev, out.shape)
        out = np.clip(out, 0, 255).astype(np.uint8)
        if self.blur_kernel > 1:
            k = self.blur_kernel if self.blur_kernel % 2 == 1 else self.blur_kernel + 1
            kernel = np.zeros((k, k), dtype=np.float32)
            kernel[k // 2, :] = 1.0 / k
            out = cv2.filter2D(out, -1, kernel)
        if self.jpeg_quality < 100:
            ok, enc = cv2.imencode('.jpg', out, [int(cv2.IMWRITE_JPEG_QUALITY), max(1, self.jpeg_quality)])
            if ok:
                out = cv2.imdecode(enc, cv2.IMREAD_COLOR)
        out_msg = bgr8_to_imgmsg(out)
        out_msg.header = msg.header
        if self.frame_delay_ms > 0:
            rospy.Timer(rospy.Duration(self.frame_delay_ms / 1000.0), lambda event: self.pub.publish(out_msg), oneshot=True)
        else:
            self.pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('image_impairment_node')
    ImageImpairmentNode()
    rospy.spin()
