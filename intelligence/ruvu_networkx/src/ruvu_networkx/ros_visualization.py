import networkx as nx

import rospy
from geometry_msgs.msg import Vector3, Pose, Quaternion
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker


def get_visualization_marker_array_msg_from_pose_graph(graph, frame_id, attribute_name="pose"):
    """
    Returns a visualization_msgs/MarkerArray from a pose graph
    :param frame_id: Frame name in the header of the message
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

    arrow_length = 1.0
    arrow_width = 0.15
    marker_array.markers += [Marker(
        header=header,
        ns="nodes_arrow",
        id=i,
        pose=pose,
        type=Marker.ARROW,
        scale=Vector3(x=arrow_length, y=arrow_width, z=arrow_width),
        color=ColorRGBA(1.0, 0.4, 0.4, 1.0),
    ) for i, pose in enumerate(graph_poses_dict.values())]

    sphere_size = 0.2
    marker_array.markers += [Marker(
        header=header,
        ns="nodes_sphere",
        id=i,
        pose=pose,
        type=Marker.SPHERE,
        scale=Vector3(x=sphere_size, y=sphere_size, z=sphere_size),
        color=ColorRGBA(0.4, 0.4, 0.8, 1.0),
    ) for i, pose in enumerate(graph_poses_dict.values())]

    shaft_diameter = 0.03
    head_diameter = 0.15
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
