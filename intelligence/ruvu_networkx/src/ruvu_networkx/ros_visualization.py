import networkx as nx
import itertools

import rospy
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3, Pose, Quaternion
from visualization_msgs.msg import MarkerArray, Marker


def get_visualization_marker_array_msg_from_pose_graph(graph, frame_id, attribute_name="pose"):
    """
    Returns a visualization_msgs/MarkerArray from a pose graph
    :param frame_id: Frame name in the header of the message
    :param graph: The networkx graph
    :param attribute_name: The pose attribute name
    """
    graph_poses = nx.get_node_attributes(graph, attribute_name)

    marker_array = MarkerArray()
    header = Header(
        frame_id=frame_id,
        stamp=rospy.Time.now()
    )
    marker_array.markers.append(Marker(
        action=Marker.DELETEALL
    ))

    node_scale = 0.2
    marker_array.markers.append(Marker(
        header=header,
        ns="nodes",
        pose=Pose(orientation=Quaternion(w=1)),
        type=Marker.SPHERE_LIST,
        scale=Vector3(x=node_scale, y=node_scale, z=node_scale),
        color=ColorRGBA(1.0, 1.0, 1.0, 1.0),
        points=[graph_poses[n].position for n in graph.nodes]
    ))

    edge_width = 0.1
    marker_array.markers.append(Marker(
        header=header,
        ns="edges",
        pose=Pose(orientation=Quaternion(w=1)),
        type=Marker.LINE_LIST,
        scale=Vector3(x=edge_width),
        color=ColorRGBA(0.0, 1.0, 1.0, 1.0),
        points=list(itertools.chain(*[(graph_poses[n1].position, graph_poses[n2].position) for n1, n2 in graph.edges])),
        colors=list(itertools.chain(*[(ColorRGBA(a=1.0), ColorRGBA(1.0, 1.0, 1.0, 1.0)) for n1, n2 in graph.edges]))
    ))

    arrow_length = 1.0
    arrow_width = 0.2
    marker_array.markers += [Marker(
        header=header,
        ns="poses",
        id=i,
        pose=graph_poses[n],
        type=Marker.ARROW,
        scale=Vector3(x=arrow_length, y=arrow_width, z=arrow_width),
        color=ColorRGBA(1.0, 0.4, 0.4, 1.0),
    ) for i, n in enumerate(graph.nodes)]

    return marker_array
