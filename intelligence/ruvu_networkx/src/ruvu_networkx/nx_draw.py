import networkx as nx


def draw_pose_graph(graph, attribute_name="pose"):
    """
    Draw a graph with a geometry_msgs/Pose graph as attribute using matplotlib
    :param graph: Graph reference
    :param attribute_name: The pose attribute name
    """
    poses = nx.get_node_attributes(graph, attribute_name)
    positions_2d = {k: (v.position.x, v.position.y) for k, v in poses.iteritems()}
    nx.draw(graph, positions_2d)
