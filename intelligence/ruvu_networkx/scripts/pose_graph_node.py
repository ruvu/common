#!/usr/bin/env python

# Copyright 2020 RUVU Robotics B.V.

from __future__ import division
import networkx as nx
import os

import rospy
import actionlib
import tf2_ros
from std_srvs.srv import Empty
from copy import deepcopy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, PoseStamped, PointStamped, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, InteractiveMarkerFeedback
from mbf_msgs.msg import GetPathAction, GetPathResult
from tf.transformations import quaternion_slerp
from future.utils import iteritems

from ruvu_networkx import ros_visualization

import math
import numpy as np


def _load_pose_graph(file_path):
    """
    Load a pose graph from a file, raise an exception if the required attributes for the nodes and edges are missing
    :param file_path: File path of the graph
    :return: The graph
    """
    try:
        graph = nx.read_yaml(file_path).to_directed()
    except AttributeError as _:
        raise ValueError("Could not read directed graph")

    # verify the graph
    if len(nx.get_node_attributes(graph, 'pose')) != len(graph.nodes()):
        raise ValueError("Every node in the graph should hold a pose attribute")
    if len(nx.get_edge_attributes(graph, 'weight')) != len(graph.edges()):
        raise ValueError("Every edge in the graph should hold a weight attribute")

    rospy.loginfo("Loaded graph with {} nodes and {} edges from {}".format(
        len(graph.nodes()),
        len(graph.edges()),
        file_path
    ))
    return graph


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


def _get_unique_node_id(graph):
    """
    Creates an unique node name to be added to the graph
    :return: The node name
    """
    return max(graph.nodes()) + 1 if len(graph.nodes()) != 0 else 1


def _get_squared_distance(p1, p2):
    """
    Calculate the squared distance between two geometry_msgs/Point
    :param p1: Point 1
    :param p2: Point 2
    :return: Squared distance
    """
    return (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2


def _get_edges_with_projected_interpolated_pose_within_tolerance(graph, position, tolerance):
    graph_poses = nx.get_node_attributes(graph, "pose")
    edges_with_projected_interpolated_pose = []

    for node1, node2 in graph.edges():
        pose1 = graph_poses[node1]
        pose2 = graph_poses[node2]

        # Edge is going from a to b and we would like to project p on this line
        a = np.array([pose1.position.x, pose1.position.y, pose1.position.z])
        b = np.array([pose2.position.x, pose2.position.y, pose2.position.z])
        p = np.array([position.x, position.y, position.z])

        line_ap = p - a
        line_ab = b - a

        # Calculate how far we are on the line, should be 0 <= factor <= 1
        projection_on_line_factor = np.dot(line_ap, line_ab) / np.dot(line_ab, line_ab)

        if 0 <= projection_on_line_factor <= 1:
            projected_point = Point(*(a + projection_on_line_factor * line_ab))

            # If the point is close enough to the passed position, we add a node with a connection to pose2
            # note that we only want to add one node for the same projection point (same edge in other direction)
            if _get_squared_distance(projected_point, position) < tolerance:
                edges_with_projected_interpolated_pose.append((
                    node1,
                    node2,
                    _get_interpolated_pose(pose1, pose2, projection_on_line_factor)
                ))

    return edges_with_projected_interpolated_pose


def _add_projected_nodes_on_edges(graph, position, tolerance):
    """
    Loop over all edges and project the position to this edge, if the distance to the edge is
    smaller than the tolerance, add a node with an edge with the same end node as the found edge.
    :param graph: Graph reference
    :param position: Position of the point that has to be projected on the edges
    :param tolerance: Max distance to edge
    """
    graph_poses = nx.get_node_attributes(graph, "pose")
    edges_with_projected_interpolated_pose = \
        _get_edges_with_projected_interpolated_pose_within_tolerance(graph, position, tolerance)

    for _, node2, interpolated_pose in edges_with_projected_interpolated_pose:
        pose2 = graph_poses[node2]

        new_node = _get_unique_node_id(graph)
        graph.add_node(new_node, pose=interpolated_pose)
        rospy.loginfo("Adding node %d with pose %s", new_node, interpolated_pose)
        graph.add_edge(new_node, node2, weight=_get_squared_distance(interpolated_pose.position, pose2.position))
        rospy.loginfo("Connecting node %d to node %d", new_node, node2)

    rospy.loginfo("Added %d projected nodes on edges", len(edges_with_projected_interpolated_pose))


def _find_shortest_path(graph, start_nodes, end_node):
    """
    Find shortest path between a number of start node candidates and an end node
    :param graph: Graph reference
    :param start_nodes: List of optional start nodes
    :param end_node: Desired end node
    :return: The shortest path if found, nx.NetworkXNoPath exception otherwise
    """
    shortest_paths = []
    for start_node in start_nodes:
        try:
            shortest_path = nx.shortest_path(graph, source=start_node, target=end_node)
            shortest_path_length = nx.shortest_path_length(graph, source=start_node, target=end_node)
        except nx.NetworkXNoPath as e:
            rospy.logdebug(e)
        else:
            if len(shortest_path) == 1:
                shortest_path *= 2  # Ensure paths of length >= 2
            shortest_paths.append((shortest_path, shortest_path_length))

    if shortest_paths:
        shortest_path, _ = min(shortest_paths, key=lambda path: path[1])
        rospy.loginfo("Performed %d searches", len(start_nodes))
        return shortest_path
    else:
        raise nx.NetworkXNoPath


def pairs(seq):
    """
    Iterate in pairs of two over an iterable

    See also https://stackoverflow.com/questions/5764782/iterate-through-pairs-of-items-in-a-python-list
    """
    i = iter(seq)
    prev = next(i)
    for item in i:
        yield prev, item
        prev = item


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
    header = Header(stamp=rospy.Time.now(), frame_id=frame_id)

    msg = Path(header=header)

    for pose1, pose2 in pairs(graph_poses[i] for i in path):
        if interpolation_distance:
            steps = math.ceil(
                math.hypot(pose2.position.x - pose1.position.x, pose2.position.y - pose1.position.y) /
                interpolation_distance
            )
            # Perform interpolation
            msg.poses += [
                PoseStamped(header=header, pose=_get_interpolated_pose(pose1, pose2, f))
                for f in np.linspace(0, 1, steps, endpoint=False)
            ]
        else:
            msg.poses.append(PoseStamped(header=header, pose=pose1))
    msg.poses.append(PoseStamped(header=header, pose=pose2))

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


class PoseGraphNode(object):
    def __init__(self, frame_id, robot_frame_id, file_path, click_timeout, interpolation_distance,
                 include_goal_pose, marker_scale):
        """
        PoseGraphNode that holds a pose graph that can be created and modified by the user. This pose graph can be used
        to search paths in euclidean space
        :param frame_id: Frame ID of the graph
        :param robot_frame_id: Robot frame id, when a path is queried from the robot's frame
        :param file_path: Where to store the graph
        :param click_timeout: Timeout between clicks when adding an edge or querying a path
        :param interpolation_distance: Pose interpolation between graph poses (when a path is calculated)
        :param include_goal_pose: Append goal pose to the end of the path or not
        :param marker_scale: The scale of the visualization markers
        """
        self._graph = nx.DiGraph()
        if os.path.isfile(file_path):
            try:
                self._graph = _load_pose_graph(file_path)
            except ValueError as e:
                rospy.logerr("Failed to load graph from file_path {}: {}".format(file_path, e))

        self._frame_id = frame_id
        self._robot_frame_id = robot_frame_id
        self._file_path = file_path
        self._last_connect_clicked_point = None
        self._last_get_path_clicked_point = None
        self._click_timeout = click_timeout
        self._interpolation_distance = interpolation_distance
        self._include_goal_pose = include_goal_pose
        self._marker_scale = marker_scale

        self._interactive_marker_server = InteractiveMarkerServer("graph_node_controls")
        self._visualization_pub = rospy.Publisher("graph_visualization", MarkerArray, queue_size=1, latch=True)
        self._last_planned_path_pub = rospy.Publisher("last_planned_path", Path, queue_size=1, latch=True)
        self._add_node_sub = rospy.Subscriber("add_node", PoseStamped, self._add_node_cb)
        self._remove_node_sub = rospy.Subscriber("remove_node", PointStamped, self._remove_node_cb)
        self._remove_edge_sub = rospy.Subscriber("remove_edge", PointStamped, self._remove_edge_cb)
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
        rospy.logwarn('store is deprecated, changes are stored automatically')
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
    def _get_sorted_nodes_within_distance(graph, point, max_distance):
        """
        Returns the node within the max_distance (based on euclidean distance)
        :param graph: The graph
        :param point: The point
        :param max_distance: Maximum distance
        :return: The nodes within the distance, [] if no nodes were found
        """
        if len(graph.nodes()) == 0:
            rospy.logerr("No nodes in graph!")
            return []

        nodes = []
        for node_id, node_data in graph.nodes(data=True):
            if _get_squared_distance(node_data['pose'].position, point) < max_distance ** 2:
                nodes.append((node_id, node_data))
        nodes = sorted(nodes, key=lambda n: _get_squared_distance(point, n[1]['pose'].position))

        return [node_id for node_id, node_data in nodes]

    def _add_node_cb(self, pose):
        """
        Callback called when adding a node to the graph using a pose stamped
        :param pose: The geometry_msgs.PoseStamped
        """
        if not self._check_frame_id(pose.header.frame_id):
            return

        self._add_pose_to_graph(pose.pose)
        nx.write_yaml(self._graph, self._file_path)

    def _remove_node_cb(self, point):
        """
        Callback fired to remove a node from the graph together with its edges
        :param point: A point near to the graph pose, the closest graph pose is removed
        """
        if not self._check_frame_id(point.header.frame_id):
            return

        self._remove_pose_from_graph(point.point)
        nx.write_yaml(self._graph, self._file_path)

    def _add_edge_cb(self, point):
        """
        Callback for adding an edge to the graph. We do hold state for memorizing the last clicked point so that we can
        connect to clicked nodes in the graph when we click on two nodes in an interval < click_timeout
        :param point: The clicked point, we will select the closest node
        """
        rospy.loginfo("add edge")
        if not self._check_frame_id(point.header.frame_id):
            return

        if self._last_connect_clicked_point \
                and (point.header.stamp - self._last_connect_clicked_point.header.stamp).to_sec() < self._click_timeout:
            self._add_edge_to_graph(self._last_connect_clicked_point.point, point.point)
            rospy.loginfo("let's connect")
            nx.write_yaml(self._graph, self._file_path)

        self._last_connect_clicked_point = point

    def _remove_edge_cb(self, point):
        """
        Callback fired to remove an edge from the graph
        :param point: A point near to the graph pose, the closest graph edge is removed
        """
        if not self._check_frame_id(point.header.frame_id):
            return

        self._remove_edge_from_graph(point.point)
        nx.write_yaml(self._graph, self._file_path)

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
            graph = deepcopy(self._graph)
            _add_projected_nodes_on_edges(graph, self._last_get_path_clicked_point.point, 1.0)

            start_nodes = self._get_sorted_nodes_within_distance(graph, self._last_get_path_clicked_point.point, 1.0)
            end_nodes = self._get_sorted_nodes_within_distance(self._graph, point.point, 1.0)
            if start_nodes and end_nodes:
                try:
                    shortest_path = _find_shortest_path(graph, start_nodes, end_nodes[0])
                except nx.NetworkXNoPath as _:
                    rospy.logwarn("No path could be found between %s and %d", start_nodes, end_nodes[0])
                    self._last_planned_path_pub.publish(_get_empty_path(self._frame_id))
                else:
                    rospy.loginfo("Found shortest path: %s", shortest_path)
                    path = _nx_path_to_nav_msgs_path(graph, shortest_path, self._frame_id)
                    self._last_planned_path_pub.publish(path)
            elif start_nodes is []:
                rospy.logwarn("No start nodes could be found")
            elif end_nodes is []:
                rospy.logwarn("No end node could be found")

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
        graph = deepcopy(self._graph)

        # Select start and end nodes
        if goal.use_start_pose:
            _add_projected_nodes_on_edges(graph, goal.start_pose.pose.position, goal.tolerance)
            start_nodes = self._get_sorted_nodes_within_distance(graph, goal.start_pose.pose.position, goal.tolerance)
            end_nodes = self._get_sorted_nodes_within_distance(self._graph, goal.target_pose.pose.position,
                                                               goal.tolerance)
        else:
            try:
                transform = self._tf_buffer.lookup_transform(self._frame_id, self._robot_frame_id, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                result.outcome = GetPathResult.TF_ERROR
                result.message = "failed to obtain transform from {} to {}".format(self._frame_id, self._robot_frame_id)
                self._get_path_as.set_aborted(result, result.message)
                return
            else:
                _add_projected_nodes_on_edges(graph, transform.transform.translation, goal.tolerance)
                start_nodes = self._get_sorted_nodes_within_distance(graph, transform.transform.translation,
                                                                     goal.tolerance)
                end_nodes = self._get_sorted_nodes_within_distance(self._graph, goal.target_pose.pose.position,
                                                                   goal.tolerance)

        # Plan the path if start and end nodes are valid
        if start_nodes and end_nodes:
            end_node = end_nodes[0]

            rospy.loginfo("Using start nodes %s and end node %s", start_nodes, end_node)

            try:
                shortest_path = _find_shortest_path(graph, start_nodes, end_node)
            except nx.NetworkXNoPath:
                result.outcome = GetPathResult.NO_PATH_FOUND
                result.message = "No path could be found between {} and {}".format(start_nodes, end_node)
                rospy.logwarn("%s", result.message)
                self._last_planned_path_pub.publish(_get_empty_path(self._frame_id))
            else:
                rospy.loginfo("Found shortest path: %s", shortest_path)
                result.path = _nx_path_to_nav_msgs_path(graph, shortest_path, self._frame_id,
                                                        interpolation_distance)

                if goal.planner == "global" and self._include_goal_pose:
                    end_pose = goal.target_pose
                    # If orientation not set (zero quaternion is invalid), use orientation from graph node pose
                    if end_pose.pose.orientation == Quaternion():
                        end_pose.pose.orientation = result.path.poses[-1].pose.orientation
                    result.path.poses.append(end_pose)

                self._last_planned_path_pub.publish(result.path)
        elif not start_nodes:
            result.outcome = GetPathResult.NO_PATH_FOUND
            result.message = "No start nodes could not be found, with tolerance {}".format(goal.tolerance)
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
        new_node = _get_unique_node_id(self._graph)

        self._graph.add_node(new_node, pose=pose)
        rospy.loginfo("Adding node {}".format(new_node))
        self._publish_graph_visualization()

    def _add_edge_to_graph(self, p1, p2):
        """
        Method for adding an edge to the graph
        :param p1: Point one (we will look for the closes pose within a tolerance region of 1.0)
        :param p2: Point two (we will look for the closes pose within a tolerance region of 1.0)
        """
        nodes1 = self._get_sorted_nodes_within_distance(self._graph, p1, 1.0)
        nodes2 = self._get_sorted_nodes_within_distance(self._graph, p2, 1.0)

        if nodes1 and nodes2:
            self._graph.add_edge(nodes1[0], nodes2[0], weight=_get_squared_distance(p1, p2))
            rospy.loginfo("Adding edge between {} and {}".format(nodes1[0], nodes2[0]))
            self._publish_graph_visualization()

    def _remove_pose_from_graph(self, point):
        """
        Method to remove a pose from the graph together with its edges
        :param point: Point (we will look for the closes pose within a tolerance region of 1.0)
        """
        nodes = self._get_sorted_nodes_within_distance(self._graph, point, 1.0)
        if nodes:
            self._graph.remove_node(nodes[0])
            rospy.loginfo("Removed node {}".format(nodes[0]))
            self._publish_graph_visualization()

    def _remove_edge_from_graph(self, point):
        """
        Method to remove an edge from the graph together
        :param point: Point (we will look for the closest edge within a tolerance region of 1.0)
        """
        edges_with_projected_interpolated_pose = \
            _get_edges_with_projected_interpolated_pose_within_tolerance(self._graph, point, 1.0)

        sorted_edges_with_poses = sorted(edges_with_projected_interpolated_pose,
                                         key=lambda n1_n2_pose: _get_squared_distance(point, n1_n2_pose[2].position))

        if sorted_edges_with_poses:
            node1, node2, _ = sorted_edges_with_poses[0]
            self._graph.remove_edge(node1, node2)

            rospy.loginfo("Removed edge between {} and {}".format(node1, node2))
            self._publish_graph_visualization()

    def _publish_graph_visualization(self):
        """
        Publish a visualization of the pose graph
        """
        self._update_interactive_markers()
        msg = ros_visualization.get_visualization_marker_array_msg_from_pose_graph(self._graph, self._frame_id,
                                                                                   self._marker_scale)
        self._visualization_pub.publish(msg)

    def _update_interactive_markers(self):
        """
        Method to update the interactive marker server based on the graph
        """
        graph_poses_dict = {str(node_id): pose for (node_id, pose) in
                            nx.get_node_attributes(self._graph, "pose").items()}
        marker_poses_dict = {name: self._interactive_marker_server.marker_contexts[name].int_marker.pose
                             for name in self._interactive_marker_server.marker_contexts}

        # Find differences between graph and existing interactive markers
        new_nodes = {node: graph_poses_dict[node] for node in graph_poses_dict if node not in marker_poses_dict}
        updated_nodes = {node: graph_poses_dict[node] for node in graph_poses_dict if
                         node in marker_poses_dict and graph_poses_dict[node] != marker_poses_dict[node]}
        removed_nodes = {node: marker_poses_dict[node] for node in marker_poses_dict if node not in graph_poses_dict}

        # Update interactive markers
        for (name, pose) in iteritems(new_nodes):
            new_marker = ros_visualization.create_3dof_marker(name, pose, self._frame_id, self._marker_scale)
            self._interactive_marker_server.insert(new_marker, self._process_marker_feedback)
        for (name, pose) in iteritems(updated_nodes):
            self._interactive_marker_server.setPose(name, pose)
        for name in removed_nodes:
            self._interactive_marker_server.erase(name)
        self._interactive_marker_server.applyChanges()

    def _process_marker_feedback(self, feedback):
        """
        Callback method for interactive marker feedback from the GUI
        :param feedback: InteractiveMarkerFeedback, containing information on the controlled interactive marker
        """
        self._graph.add_node(int(feedback.marker_name), pose=feedback.pose)  # Update existing node's pose
        self._publish_graph_visualization()
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            nx.write_yaml(self._graph, self._file_path)


if __name__ == "__main__":
    rospy.init_node("pose_graph_node")
    try:
        pgn = PoseGraphNode(
            rospy.get_param("~frame_id", "map"),
            rospy.get_param("~robot_frame_id", "base_link"),
            rospy.get_param("~file_path", "/tmp/pose_graph.yaml"),
            rospy.get_param("~click_timeout", 5.0),
            rospy.get_param("~interpolation_distance", 0.2),
            rospy.get_param("~include_goal_pose", True),
            rospy.get_param("~marker_scale", 1)
        )
    except Exception:
        # if we don't catch this exception, the node hangs because other threads have been started
        rospy.logfatal("Pose graph initialization failed")
        rospy.signal_shutdown('exception during initialization')
        raise
    else:
        rospy.spin()
