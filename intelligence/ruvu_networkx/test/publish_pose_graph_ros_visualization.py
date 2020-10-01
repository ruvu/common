#!/usr/bin/env python

# Copyright 2020 RUVU Robotics B.V.

import networkx as nx
import argparse

import rospy
from visualization_msgs.msg import MarkerArray

from ruvu_networkx import ros_visualization

parser = argparse.ArgumentParser("Publish pose graph as visualization_msgs/MarkerArray")
parser.add_argument("input_file")
parser.add_argument("--frame_id", default="map")
args = parser.parse_args()

rospy.init_node("publish_pose_graph_ros_visualization")

msg = ros_visualization.get_visualization_marker_array_msg_from_pose_graph(nx.read_yaml(args.input_file), args.frame_id)
publisher = rospy.Publisher("graph_visualization", MarkerArray, queue_size=1, latch=True)
publisher.publish(msg)

rospy.loginfo("Publishing latched graph visualization ...")
rospy.spin()
