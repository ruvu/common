#!/usr/bin/env python

# Copyright 2020 RUVU Robotics B.V.

from geometry_msgs.msg import PoseStamped, TransformStamped


def pose_stamped_to_transform_stamped(msg, child_frame_id=""):
    """
    Convert a TransformStamped to a PoseStamped message
    :param msg: geometry_msgs/PoseStamped
    :param child_frame_id: String
    :return: geometry_msgs/TransformStamped
    """
    assert isinstance(msg, PoseStamped)
    ts = TransformStamped()
    ts.header = msg.header
    ts.child_frame_id = child_frame_id
    ts.transform.translation.x = msg.pose.position.x
    ts.transform.translation.y = msg.pose.position.y
    ts.transform.translation.z = msg.pose.position.z
    ts.transform.rotation = msg.pose.orientation
    return ts
