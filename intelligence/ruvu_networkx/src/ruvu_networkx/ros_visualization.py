# Copyright 2020 RUVU Robotics B.V.

import math
import networkx as nx
import rospy
from geometry_msgs.msg import Vector3, Pose, Quaternion, Point
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker, InteractiveMarkerControl, InteractiveMarker


def get_visualization_marker_array_msg_from_pose_graph(graph, frame_id, marker_scale, attribute_name="pose"):
    """
    Returns a visualization_msgs/MarkerArray from a pose graph
    :param frame_id: Frame name in the header of the message
    :param marker_scale: The scale of the visualization marker
    :param graph: The networkx graph
    :param attribute_name: The pose attribute name
    """
    graph_poses_dict = nx.get_node_attributes(graph, attribute_name)
    graph_edges_pose_pairs = [(graph_poses_dict[n1], graph_poses_dict[n2]) for n1, n2 in graph.edges()]

    marker_array = MarkerArray()
    header = Header(
        frame_id=frame_id,
        stamp=rospy.Time.now()
    )

    marker_array.markers.append(Marker(
        action=Marker.DELETEALL
    ))

    sphere_size = 0.2 * marker_scale
    marker_array.markers += [Marker(
        header=header,
        ns="nodes_sphere",
        id=i,
        pose=pose,
        type=Marker.SPHERE,
        scale=Vector3(x=sphere_size, y=sphere_size, z=sphere_size),
        color=ColorRGBA(0.4, 0.4, 0.8, 1.0),
    ) for i, pose in enumerate(graph_poses_dict.values())]

    shaft_diameter = 0.03 * marker_scale
    head_diameter = 0.15 * marker_scale
    marker_array.markers += [Marker(
        header=header,
        ns="edges_arrow",
        id=i,
        pose=Pose(orientation=Quaternion(w=1)),
        type=Marker.ARROW,
        scale=Vector3(x=shaft_diameter, y=head_diameter),
        color=ColorRGBA(0.95, 0.95, 0.95, 1.0),
        points=[pose.position for pose in pose_pair]
    ) for i, pose_pair in enumerate(graph_edges_pose_pairs)]

    return marker_array


def _create_marker_arrow(marker_scale):
    """
    Creates an arrow marker, scaled to match the interactive marker control arrows
    :param marker_scale: The scale of the visualization marker
    """
    marker = Marker()
    marker.type = Marker.ARROW
    marker.scale.x = marker_scale / 8.0
    marker.scale.y = marker_scale / 4.5
    marker.scale.z = marker_scale / 5.0
    marker.points.append(Point())
    marker.points.append(Point(x=marker_scale * 0.9))
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    return marker


def _create_marker_control(control_name, control_mode, orientation):
    control = InteractiveMarkerControl()
    control.orientation = orientation
    control.name = control_name
    control.interaction_mode = control_mode
    return control


def create_3dof_marker(name, pose, frame_id, marker_scale):
    """
    Creates an interactive marker which can be translated along in x and y and rotated around z.
    :param name: Name of the marker
    :param pose: Pose of the marker
    :param frame_id: Frame_id of the pose
    :param marker_scale: Scale size of the visualization
    """
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.pose = pose
    int_marker.scale = marker_scale
    int_marker.name = name
    int_marker.description = "Node " + name

    length = 1 / math.sqrt(2)

    # Move on xy plane
    control = _create_marker_control('move_xy_plane', InteractiveMarkerControl.MOVE_PLANE,
                                     Quaternion(y=length, w=length))
    control.markers.append(_create_marker_arrow(int_marker.scale))
    control.always_visible = True
    int_marker.controls.append(control)

    # Move along x-axis
    int_marker.controls.append(
        _create_marker_control('move_x', InteractiveMarkerControl.MOVE_AXIS, Quaternion(x=length, w=length)))

    # Move along y-axis
    int_marker.controls.append(
        _create_marker_control('move_y', InteractiveMarkerControl.MOVE_AXIS, Quaternion(z=length, w=length)))

    # Rotate around z-axis
    int_marker.controls.append(
        _create_marker_control('rotate_z', InteractiveMarkerControl.ROTATE_AXIS, Quaternion(y=length, w=length)))
    return int_marker
