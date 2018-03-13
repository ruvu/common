import networkx
from mock import patch
from ruvu_networkx.ros_visualization import get_visualization_marker_array_msg_from_pose_graph


@patch('rospy.Time.now')
def test_empty_graph(now):
    now.return_value = 3
    G = networkx.Graph()
    m = get_visualization_marker_array_msg_from_pose_graph(G, 'frame_id')
    assert m
