#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped, TransformStamped


def transform_stamped_to_pose_stamped(msg):
    """
    Convert a TransformStamped to a PoseStamped message
    :param msg: geometry_msgs/TransformStamped
    :return: geometry_msgs/PoseStamped
    """
    assert isinstance(msg, TransformStamped)
    ps = PoseStamped()
    ps.header = msg.header
    ps.pose.position.x = msg.transform.translation.x
    ps.pose.position.y = msg.transform.translation.y
    ps.pose.position.z = msg.transform.translation.z
    ps.pose.orientation = msg.transform.rotation
    return ps
