#!/usr/bin/env python

import networkx as nx
import os

import rospy
import actionlib
from std_srvs.srv import Empty
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from mbf_msgs.msg import GetPathAction, GetPathResult

from ruvu_networkx import ros_visualization


def _nx_path_to_nav_msgs_path(graph, path, frame_id):
    """
    Convert a path in the graph to a nav_msgs/Path
    :param graph: The graph
    :param path: The path
    :param frame_id: The frame id that should be used in the resulting message
    :return: The nav_msgs/Path
    """
    graph_poses = nx.get_node_attributes(graph, "pose")
    header = Header(
        stamp=rospy.Time.now(),
        frame_id=frame_id
    )
    return Path(
        header=header,
        poses=[PoseStamped(
            header=header,
            pose=graph_poses[p]
        ) for p in path]
    )


def _get_squared_distance(p1, p2):
    """
    Calculate the squared distance between two geometry_msgs/Point
    :param p1: Point 1
    :param p2: Point 2
    :return: Squared distance
    """
    return (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2


class PoseGraphNode(object):
    def __init__(self, frame_id, file_path, edge_connect_timeout):
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
        self._last_clicked_point = None
        self._edge_connect_timeout = edge_connect_timeout

        self._visualization_pub = rospy.Publisher("graph_visualization", MarkerArray, queue_size=1, latch=True)
        self._last_planner_path_pub = rospy.Publisher("last_planned_path", Path, queue_size=1, latch=True)
        self._add_node_sub = rospy.Subscriber("add_node", PoseStamped, self._add_node_cb)
        self._remove_node_sub = rospy.Subscriber("remove_node", PointStamped, self._remove_node_cb)
        self._add_edge_sub = rospy.Subscriber("add_edge", PointStamped, self._add_edge_cb)

        self._clear_service = rospy.Service("clear", Empty, self._clear_srv)
        self._store_service = rospy.Service("store", Empty, self._store_srv)

        self._get_path_as = actionlib.SimpleActionServer("get_path", GetPathAction,
                                                         execute_cb=self._get_path_action_cb, auto_start=False)
        self._get_path_as.start()

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

    def _get_closest_node(self, point, tolerance=None):
        if len(self._graph.nodes) == 0:
            return None

        closest_node, closest_node_data = min(self._graph.nodes(data=True),
                                              key=lambda nd: _get_squared_distance(point, nd[1]['pose'].position))

        if tolerance is not None and _get_squared_distance(closest_node_data['pose'].position, point) > tolerance ** 2:
            return None

        return closest_node

    def _get_unique_node_id(self):
        return max(self._graph.nodes) + 1 if len(self._graph.nodes) != 0 else 1

    def _add_node_cb(self, pose):
        if not self._check_frame_id(pose.header.frame_id):
            return

        self._add_pose_to_graph(pose.pose)

    def _remove_node_cb(self, point):
        if not self._check_frame_id(point.header.frame_id):
            return

        self._remove_pose_from_graph(point.point)

    def _add_edge_cb(self, point):
        if not self._check_frame_id(point.header.frame_id):
            return

        if self._last_clicked_point \
                and (rospy.Time.now() - self._last_clicked_point.header.stamp).to_sec() < self._edge_connect_timeout:
            self._add_edge_to_graph(self._last_clicked_point.point, point.point)

        self._last_clicked_point = point

    def _get_path_action_cb(self, goal):
        result = GetPathResult()

        # Check whether the goal is valid before planning the path
        if not goal.use_start_pose:
            result.outcome = GetPathResult.INVALID_START
            result.message = "use_start_pose should be set to true"
        elif goal.start_pose.header.frame_id != self._frame_id:
            result.outcome = GetPathResult.INVALID_START
            result.message = "start_pose frame_id != {}".format(self._frame_id)
        elif goal.target_pose.header.frame_id != self._frame_id:
            result.outcome = GetPathResult.INVALID_GOAL
            result.message = "target_pose frame_id != {}".format(self._frame_id)
        elif len(self._graph.nodes) == 0:
            result.outcome = GetPathResult.NOT_INITIALIZED
            result.message = "no nodes in the graph, is it initialized properly?"
        else:
            start_node = self._get_closest_node(goal.start_pose.pose.position, goal.tolerance)
            end_node = self._get_closest_node(goal.target_pose.pose.position, goal.tolerance)
            if start_node and end_node:
                try:
                    shortest_path = nx.shortest_path(self._graph, source=start_node, target=end_node)
                except nx.NetworkXNoPath as e:
                    result.outcome = GetPathResult.NO_PATH_FOUND
                    result.message = e.message
                else:
                    result.path = _nx_path_to_nav_msgs_path(self._graph, shortest_path, self._frame_id)
                    self._last_planner_path_pub.publish(result.path)
            else:
                result.outcome = GetPathResult.NO_PATH_FOUND
                result.message = "no start or end node could be found, with tolerance {}".format(goal.tolerance)

        if result.outcome == GetPathResult.SUCCESS:
            self._get_path_as.set_succeeded(result, result.message)
        else:
            self._get_path_as.set_aborted(result, result.message)

    def _add_pose_to_graph(self, pose):
        new_node = self._get_unique_node_id()

        self._graph.add_node(new_node, pose=pose)
        rospy.loginfo("Adding node {}".format(new_node))

        self._publish_graph_visualization()

    def _add_edge_to_graph(self, p1, p2):
        n1 = self._get_closest_node(p1, tolerance=1.0)
        n2 = self._get_closest_node(p2, tolerance=1.0)
        if n1 and n2:
            self._graph.add_edge(n1, n2, weight=_get_squared_distance(p1, p2))
            rospy.loginfo("Adding edge between {} and {}".format(n1, n2))
            self._publish_graph_visualization()

    def _remove_pose_from_graph(self, point):
        n = self._get_closest_node(point, tolerance=1.0)
        if n:
            self._graph.remove_node(n)
            rospy.loginfo("Removed node {}".format(n))
            self._publish_graph_visualization()

    def _publish_graph_visualization(self):
        msg = ros_visualization.get_visualization_marker_array_msg_from_pose_graph(self._graph, self._frame_id)
        self._visualization_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("pose_graph_node")
    pgn = PoseGraphNode(
        rospy.get_param("frame_id", "map"),
        rospy.get_param("file_path", "/tmp/pose_graph.yaml"),
        rospy.get_param("edge_connect_timeout", 5.0)
    )
    rospy.spin()
