#!/usr/bin/env python

import networkx as nx

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

from ruvu_networkx import ros_visualization


def pose_callback(msg):
    global G, FRAME_ID, NODE_ID

    # Take frame ID of the header
    if FRAME_ID is None:
        FRAME_ID = msg.header.frame_id
    else:
        if msg.header.frame_id != FRAME_ID:
            rospy.logerr("Invalid frame_id {}, graph frame_id is {}".format(msg.header.frame_id, FRAME_ID))
            return

    NODE_ID += 1

    G.add_node(NODE_ID, pose=msg.pose)
    if len(G.nodes) > 1:
        G.add_edge(NODE_ID - 1, NODE_ID)

    msg = ros_visualization.get_visualization_marker_array_msg_from_pose_graph(G, FRAME_ID)
    publisher.publish(msg)


NODE_ID = 1
G = nx.DiGraph()
FRAME_ID = None


rospy.init_node("pose_graph_node")

publisher = rospy.Publisher("graph_visualization", MarkerArray, queue_size=1, latch=True)
subscriber = rospy.Subscriber("add_graph_pose", PoseStamped, pose_callback)

rospy.loginfo("Publishing latched graph visualization ...")
rospy.loginfo("Use RVIZ to add poses to the graph the 2d nav goal tool with topic add_graph_pose")
rospy.spin()
