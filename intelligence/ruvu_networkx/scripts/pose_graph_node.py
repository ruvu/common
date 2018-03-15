#!/usr/bin/env python

import networkx as nx
import os

import rospy
import actionlib
import tf2_ros
from std_srvs.srv import Empty
from copy import copy, deepcopy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, PoseStamped, PointStamped, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from mbf_msgs.msg import GetPathAction, GetPathResult
from tf.transformations import quaternion_slerp

from ruvu_networkx import ros_visualization

import math
import numpy as np


def _get_interpolated_pose(pose1, pose2, fraction):
    """
    Perform pose interpolation between two geometry_msgs.Pose
    :param pose1: First pose
    :param pose2: Second pose
    :param fraction: Fraction from 0..1 (0 means pose1)
    :return: The interpolated pose
    """
    return Pose(
        position=Point(
            x=pose1.position.x * (1 - fraction) + pose2.position.x * fraction,
            y=pose1.position.y * (1 - fraction) + pose2.position.y * fraction,
            z=pose1.position.z * (1 - fraction) + pose2.position.z * fraction
        ),
        orientation=Quaternion(
            *quaternion_slerp(pose1.orientation.__getstate__(), pose2.orientation.__getstate__(), fraction)
        )
    )


def _add_projected_nodes_on_edges(graph, position, tolerance):
    """
    Loop over all edges and project the position to this edge, if the distance to the edge is
    smaller than the tolerance, add a node with an edge with the same end node as the found edge.
    :param graph: Graph reference
    :param position: Position of the point that has to be projected on the edges
    :param tolerance: Max distance to edge
    """
    import ipdb; ipdb.set_trace()


def _nx_path_to_nav_msgs_path(graph, path, frame_id, interpolation_distance=0):
    """
    Convert a path in the graph to a nav_msgs/Path
    :param graph: The graph
    :param path: The path
    :param frame_id: The frame id that should be used in the resulting message
    :param interpolation_distance: Step size (position) that is used for interpolation, zero means no interpolation
    :return: The nav_msgs/Path
    """
    graph_poses = nx.get_node_attributes(graph, "pose")
    header = Header(
        stamp=rospy.Time.now(),
        frame_id=frame_id
    )

    msg = Path(header=header)

    last_idx = None
    for idx in path:
        pose = graph_poses[idx]
        if last_idx is not None and interpolation_distance:
            last_pose = graph_poses[last_idx]

            step_size = 1. / int(math.hypot(pose.position.x - last_pose.position.x,
                                            pose.position.y - last_pose.position.y) / interpolation_distance)

            # Perform interpolation
            msg.poses += [PoseStamped(header=header, pose=_get_interpolated_pose(last_pose, pose, f))
                          for f in np.arange(step_size, 1.0, step_size)]
        else:
            msg.poses.append(PoseStamped(
                header=header,
                pose=pose
            ))
        last_idx = idx

    return msg


def _get_empty_path(frame_id):
    """
    Returns an empty path
    :param frame_id: The frame id
    :return: The path
    """
    return Path(
        header=Header(
            stamp=rospy.Time.now(),
            frame_id=frame_id
        )
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
    def __init__(self, frame_id, robot_frame_id, file_path, click_timeout, interpolation_distance):
        """
        PoseGraphNode that holds a pose graph that can be created and modified by the user. This pose graph can be used
        to search paths in euclidean space
        :param frame_id: Frame ID of the graph
        :param robot_frame_id: Robot frame id, when a path is queried from the robot's frame
        :param file_path: Where to store the graph
        :param click_timeout: Timeout between clicks when adding an edge or querying a path
        :param interpolation_distance: Pose interpolation between graph poses (when a path is calculated)
        """
        self._graph = nx.DiGraph()
        if os.path.isfile(file_path):
            self._graph = nx.read_yaml(file_path)
            rospy.loginfo("Loaded graph with {} nodes and {} edges from {}".format(
                len(self._graph.nodes()),
                len(self._graph.edges()),
                file_path
            ))

        self._frame_id = frame_id
        self._robot_frame_id = robot_frame_id
        self._file_path = file_path
        self._last_connect_clicked_point = None
        self._last_get_path_clicked_point = None
        self._click_timeout = click_timeout
        self._interpolation_distance = interpolation_distance

        self._visualization_pub = rospy.Publisher("graph_visualization", MarkerArray, queue_size=1, latch=True)
        self._last_planner_path_pub = rospy.Publisher("last_planned_path", Path, queue_size=1, latch=True)
        self._add_node_sub = rospy.Subscriber("add_node", PoseStamped, self._add_node_cb)
        self._remove_node_sub = rospy.Subscriber("remove_node", PointStamped, self._remove_node_cb)
        self._add_edge_sub = rospy.Subscriber("add_edge", PointStamped, self._add_edge_cb)
        self._get_path_sub = rospy.Subscriber("get_path", PointStamped, self._get_path_cb)

        self._clear_service = rospy.Service("clear", Empty, self._clear_srv)
        self._store_service = rospy.Service("store", Empty, self._store_srv)

        self._get_path_as = actionlib.SimpleActionServer("get_path", GetPathAction,
                                                         execute_cb=self._get_path_action_cb, auto_start=False)
        self._get_path_as.start()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._publish_graph_visualization()
        rospy.loginfo("PoseGraphNode initialized")

    def _clear_srv(self, _):
        """
        Callback received when the clear service is called, it will clear the pose graph
        """
        self._graph.clear()
        rospy.loginfo("Cleared graph")
        self._publish_graph_visualization()
        return {}

    def _store_srv(self, _):
        """
        Callback received when the store service is called, it will store the graph to file (file path parameter)
        """
        nx.write_yaml(self._graph, self._file_path)
        rospy.loginfo("Stored graph to {}".format(self._file_path))
        return {}

    def _check_frame_id(self, frame_id):
        """
        Checks whether the param frame id equals the graph frame id
        :param frame_id: The frame id to check
        :return: True if correct, False otherwise
        """
        if frame_id != self._frame_id:
            rospy.logerr("Received pose with invalid frame_id {}. Graph frame_id = {}".format(frame_id, self._frame_id))
            return False
        return True

    @staticmethod
    def _get_closest_node(graph, point, tolerance=None):
        """
        Returns the closes node (based on euclidean distance)
        :param graph: The graph
        :param point: The point
        :param tolerance: Maximum tolerance
        :return: The closes node id, None if no node was found
        """
        if len(graph.nodes()) == 0:
            return None

        closest_node, closest_node_data = min(graph.nodes(data=True),
                                              key=lambda nd: _get_squared_distance(point, nd[1]['pose'].position))

        if tolerance is not None and _get_squared_distance(closest_node_data['pose'].position, point) > tolerance ** 2:
            return None

        return closest_node

    def _get_unique_node_id(self):
        """
        Creates an unique node name to be added to the graph
        :return: The node name
        """
        return max(self._graph.nodes()) + 1 if len(self._graph.nodes()) != 0 else 1

    def _add_node_cb(self, pose):
        """
        Callback called when adding a node to the graph using a pose stamped
        :param pose: The geometry_msgs.PoseStamped
        """
        if not self._check_frame_id(pose.header.frame_id):
            return

        self._add_pose_to_graph(pose.pose)

    def _remove_node_cb(self, point):
        """
        Callback fired to remove a node from the graph together with its edges
        :param point: A point near to the graph pose, the closest graph pose is removed
        """
        if not self._check_frame_id(point.header.frame_id):
            return

        self._remove_pose_from_graph(point.point)

    def _add_edge_cb(self, point):
        """
        Callback for adding an edge to the graph. We do hold state for memorizing the last clicked point so that we can
        connect to clicked nodes in the graph when we click on two nodes in an interval < click_timeout
        :param point: The clicked point, we will select the closest node
        """
        if not self._check_frame_id(point.header.frame_id):
            return

        if self._last_connect_clicked_point \
                and (rospy.Time.now() - self._last_connect_clicked_point.header.stamp).to_sec() < self._click_timeout:
            self._add_edge_to_graph(self._last_connect_clicked_point.point, point.point)

        self._last_connect_clicked_point = point

    def _get_path_cb(self, point):
        """
        Callback for planning a route through the graph. We do hold state for memorizing the last clicked point so that
        we can plan from one point to another when we click on two nodes in an interval < click_timeout
        :param point: The clicked point, we will select the closest node
        """
        if not self._check_frame_id(point.header.frame_id):
            return

        if self._last_get_path_clicked_point \
                and (rospy.Time.now() - self._last_get_path_clicked_point.header.stamp).to_sec() < self._click_timeout:
            start_node = self._get_closest_node(self._graph, self._last_get_path_clicked_point.point, tolerance=1.0)
            end_node = self._get_closest_node(self._graph, point.point, tolerance=1.0)
            if start_node is not None and end_node is not None:
                try:
                    shortest_path = nx.shortest_path(self._graph, source=start_node, target=end_node)
                except nx.NetworkXNoPath as _:
                    self._last_planner_path_pub.publish(_get_empty_path(self._frame_id))
                else:
                    path = _nx_path_to_nav_msgs_path(self._graph, shortest_path, self._frame_id)
                    self._last_planner_path_pub.publish(path)

        self._last_get_path_clicked_point = point

    def _check_goal_validity(self, goal, result):
        if not goal.planner:
            goal.planner = 'topological'

        if goal.use_start_pose and goal.start_pose.header.frame_id != self._frame_id:
            result.outcome = GetPathResult.INVALID_START
            result.message = "start_pose frame_id != {}".format(self._frame_id)
            return False

        if goal.target_pose.header.frame_id != self._frame_id:
            result.outcome = GetPathResult.INVALID_GOAL
            result.message = "target_pose frame_id != {}".format(self._frame_id)
            return False

        return True

    def _get_path_action_cb(self, goal):
        """
        The get path action goal callback
        :param goal: The goal that describes the end contraint, the start position and what planner to use
        """
        result = GetPathResult()

        start_node = None
        end_node = None

        # Check whether the goal is valid before planning the path
        if not self._check_goal_validity(goal, result):
            self._get_path_as.set_aborted(result, result.message)
            return

        # Check if the graph is valid
        if len(self._graph.nodes()) == 0:
            result.outcome = GetPathResult.NOT_INITIALIZED
            result.message = "no nodes in the graph, is it initialized properly?"
            self._get_path_as.set_aborted(result, result.message)
            return

        # Set interpolation distance based on selected planner
        if goal.planner == "topological":
            interpolation_distance = 0
        else:  # We can safely assume goal.planner == "global", as we checked the goal validity already.
            interpolation_distance = self._interpolation_distance

        # Make a copy of the graph so that we can alter the planning graph (adding projected nodes)
        graph = copy(self._graph)

        # Select start and end nodes
        if goal.use_start_pose:
            _add_projected_nodes_on_edges(graph, goal.start_pose.pose.position, goal.tolerance)
            start_node = self._get_closest_node(graph, goal.start_pose.pose.position, goal.tolerance)
            end_node = self._get_closest_node(graph, goal.target_pose.pose.position, goal.tolerance)
        else:
            _add_projected_nodes_on_edges(graph, goal.start_pose.pose.position, goal.tolerance)
            try:
                transform = self._tf_buffer.lookup_transform(self._frame_id, self._robot_frame_id, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                result.outcome = GetPathResult.TF_ERROR
                result.message = "failed to obtain transform from {} to {}".format(self._frame_id, self._robot_frame_id)
            else:
                start_node = self._get_closest_node(graph, transform.transform.translation, goal.tolerance)
                end_node = self._get_closest_node(graph, goal.target_pose.pose.position, goal.tolerance)

        # Plan the path if start and end nodes are valid
        if start_node is not None and end_node is not None:
            try:
                shortest_path = nx.shortest_path(graph, source=start_node, target=end_node)
            except nx.NetworkXNoPath as e:
                result.outcome = GetPathResult.NO_PATH_FOUND
                result.message = e.message
                self._last_planner_path_pub.publish(_get_empty_path(self._frame_id))
            else:
                result.path = _nx_path_to_nav_msgs_path(graph, shortest_path, self._frame_id,
                                                        interpolation_distance)

                if goal.planner == "global":
                    end_pose = goal.target_pose
                    # If orientation not set (zero quaternion is invalid), use orientation from graph node pose
                    if end_pose.pose.orientation == Quaternion():
                        end_pose.pose.orientation = result.path.poses[-1].pose.orientation
                    result.path.poses.append(end_pose)

                self._last_planner_path_pub.publish(result.path)
        elif start_node is None:
            result.outcome = GetPathResult.NO_PATH_FOUND
            result.message = "Start node could not be found, with tolerance {}".format(goal.tolerance)
        else:
            result.outcome = GetPathResult.NO_PATH_FOUND
            result.message = "End node could not be found, with tolerance {}".format(goal.tolerance)

        if result.outcome == GetPathResult.SUCCESS:
            self._get_path_as.set_succeeded(result, result.message)
        else:
            self._get_path_as.set_aborted(result, result.message)

    def _add_pose_to_graph(self, pose):
        """
        Method for adding a pose to the graph
        :param pose: The pose
        """
        new_node = self._get_unique_node_id()

        self._graph.add_node(new_node, pose=pose)
        rospy.loginfo("Adding node {}".format(new_node))

        self._publish_graph_visualization()

    def _add_edge_to_graph(self, p1, p2):
        """
        Method for adding an edge to the graph
        :param p1: Point one (we will look for the closes pose within a tolerance region of 1.0)
        :param p2: Point two (we will look for the closes pose within a tolerance region of 1.0)
        """
        n1 = self._get_closest_node(self._graph, p1, tolerance=1.0)
        n2 = self._get_closest_node(self._graph, p2, tolerance=1.0)
        if n1 is not None and n2 is not None:
            self._graph.add_edge(n1, n2, weight=_get_squared_distance(p1, p2))
            rospy.loginfo("Adding edge between {} and {}".format(n1, n2))
            self._publish_graph_visualization()

    def _remove_pose_from_graph(self, point):
        """
        Method to remove a pose from the graph together with its edges
        :param point: Point (we will look for the closes pose within a tolerance region of 1.0)
        """
        n = self._get_closest_node(self._graph, point, tolerance=1.0)
        if n is not None:
            self._graph.remove_node(n)
            rospy.loginfo("Removed node {}".format(n))
            self._publish_graph_visualization()

    def _publish_graph_visualization(self):
        """
        Publish a visualization of the pose graph
        """
        msg = ros_visualization.get_visualization_marker_array_msg_from_pose_graph(self._graph, self._frame_id)
        self._visualization_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("pose_graph_node")
    try:
        pgn = PoseGraphNode(
            rospy.get_param("~frame_id", "map"),
            rospy.get_param("~robot_frame_id", "base_link"),
            rospy.get_param("~file_path", "/tmp/pose_graph.yaml"),
            rospy.get_param("~click_timeout", 5.0),
            rospy.get_param("~interpolation_distance", 0.2)
        )
    except Exception:
        # if we don't catch this exception, the node hangs because other threads have been started
        rospy.signal_shutdown('exception during initialization')
        raise
    else:
        rospy.spin()
