#!/usr/bin/env python

import networkx as nx
import os

import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

from ruvu_networkx import ros_visualization


def _get_squared_distance(p1, p2):
    """
    Calculate the squared distance between two geometry_msgs/Point
    :param p1: Point 1
    :param p2: Point 2
    :return: Squared distance
    """
    return (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2


class PoseGraphNode(object):
    def __init__(self, frame_id, file_path):
        self._graph = nx.DiGraph()
        if os.path.isfile(file_path):
            self._graph = nx.read_yaml(file_path)
            rospy.loginfo("Loaded graph with {} nodes and {} edges from {}".format(
                len(self._graph.nodes),
                len(self._graph.edges),
                file_path
            ))

        self._frame_id = frame_id
        self._file_path = file_path
        self._last_added_node = None

        self._visualization_pub = rospy.Publisher("graph_visualization", MarkerArray, queue_size=1, latch=True)
        self._add_to_closest_sub = rospy.Subscriber("add_pose_to_closest_node", PoseStamped, self._add_to_closest_cb)
        self._add_to_last_added_sub = rospy.Subscriber("add_pose_to_last_added_node", PoseStamped,
                                                       self._add_to_last_added_cb)

        self._clear_service = rospy.Service("clear", Empty, self._clear_srv)
        self._store_service = rospy.Service("store", Empty, self._store_srv)

        self._publish_graph_visualization()
        rospy.loginfo("PoseGraphNode initialized")

    def _clear_srv(self, req):
        self._graph.clear()
        rospy.loginfo("Cleared graph")
        self._publish_graph_visualization()
        return {}

    def _store_srv(self, req):
        nx.write_yaml(self._graph, self._file_path)
        rospy.loginfo("Stored graph to {}".format(self._file_path))
        return {}

    def _check_frame_id(self, frame_id):
        if frame_id != self._frame_id:
            rospy.logerr("Received pose with invalid frame_id {}. Graph frame_id = {}".format(frame_id, self._frame_id))
            return False
        return True

    def _get_closest_node(self, pose):
        if len(self._graph.nodes) == 0:
            return None

        closest_node, _ = min(self._graph.nodes(data=True),
                              key=lambda nd: _get_squared_distance(pose.position, nd[1]['pose'].position))
        return closest_node

    def _get_unique_node_id(self):
        return max(self._graph.nodes) + 1 if len(self._graph.nodes) != 0 else 1

    def _add_to_closest_cb(self, pose):
        if not self._check_frame_id(pose.header.frame_id):
            return

        closest_node = self._get_closest_node(pose.pose)
        self._add_pose_to_graph(pose.pose, closest_node)

    def _add_to_last_added_cb(self, pose):
        if not self._check_frame_id(pose.header.frame_id):
            return

        self._add_pose_to_graph(pose.pose, self._last_added_node)

    def _add_pose_to_graph(self, pose, connected_node=None):
        new_node = self._get_unique_node_id()

        self._graph.add_node(new_node, pose=pose)
        rospy.loginfo("Adding node {}".format(new_node))

        if connected_node:
            rospy.loginfo("Adding edge between {} and {}".format(connected_node, new_node))
            self._graph.add_edge(connected_node, new_node)

        self._publish_graph_visualization()

        self._last_added_node = new_node

    def _publish_graph_visualization(self):
        msg = ros_visualization.get_visualization_marker_array_msg_from_pose_graph(self._graph, self._frame_id)
        self._visualization_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("pose_graph_node")
    pgn = PoseGraphNode(
        rospy.get_param("frame_id", "map"),
        rospy.get_param("file_path", "/tmp/pose_graph.yaml")
    )
    rospy.spin()
