import os
import argparse
import rosbag

parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
parser.add_argument("input_bag", help="Input ROS bag.")
parser.add_argument("output_bag", help="Output ROS bag.")

args = parser.parse_args()

input_bag = rosbag.Bag(args.input_bag, "r")
output_bag = rosbag.Bag(args.output_bag, 'w')

for topic, msg, t in input_bag.read_messages():
    if topic == "/apollo/sensor/velodyne64/compensator/PointCloud2":
        output_bag.write("/apollo/sensor/velodyne64/VelodyneScanUnified	", msg)
    else:
        output_bag.write(topic, msg)

input_bag.close()
output_bag.close()
