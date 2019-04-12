import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
parser.add_argument("input_bag", help="Input ROS bag.")
parser.add_argument("output_bag", help="Output ROS bag.")

args = parser.parse_args()

input_bag = rosbag.Bag(args.input_bag, "r")
output_bag = rosbag.Bag(args.output_bag, 'w')

bridge = CvBridge()
for topic, msg, t in input_bag.read_messages():
    if topic == "/apollo/sensor/camera/traffic/image_long":
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        scaled_img = cv2.resize(img, (1920, 1080))
        img_out = cv2.cvtColor(scaled_img, cv2.COLOR_BGR2YUV)
        scaled_msg = bridge.cv2_to_imgmsg(img_out)
        output_bag.write(topic, scaled_msg)
    elif topic == "/apollo/sensor/camera/traffic/image_short":
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        scaled_img = cv2.resize(img, (1920, 1080))
        scaled_img_bgr = cv2.cvtColor(scaled_img, cv2.COLOR_GRAY2BGR)
        img_out = cv2.cvtColor(scaled_img_bgr, cv2.COLOR_BGR2YUV)
        scaled_msg = bridge.cv2_to_imgmsg(img_out)
        output_bag.write(topic, scaled_msg)
    else:
        output_bag.write(topic, msg)

input_bag.close()
output_bag.close()
