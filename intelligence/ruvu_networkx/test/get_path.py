#!/usr/bin/env python

# Copyright 2020 RUVU Robotics B.V.

import argparse

import rospy
import actionlib
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point
import mbf_msgs.msg


def get_path_client(start_point, target_point, frame_id, tolerance):
    client = actionlib.SimpleActionClient('get_path', mbf_msgs.msg.GetPathAction)
    client.wait_for_server()

    header = Header(frame_id=frame_id, stamp=rospy.Time.now())

    goal = mbf_msgs.msg.GetPathGoal(
        use_start_pose=True,
        start_pose=PoseStamped(
            header=header,
            pose=Pose(
                position=start_point
            )
        ),
        target_pose=PoseStamped(
            header=header,
            pose=Pose(
                position=target_point
            )
        ),
        tolerance=tolerance
    )

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser("Get path from the pose graph")
        parser.add_argument("x1", type=float)
        parser.add_argument("y1", type=float)
        parser.add_argument("x2", type=float)
        parser.add_argument("y2", type=float)
        parser.add_argument("--frame_id", default="map")
        parser.add_argument("--tolerance", default=1.0, type=float)

        args = parser.parse_args()

        rospy.init_node('get_path')
        result = get_path_client(Point(x=args.x1, y=args.y1),
                                 Point(x=args.x2, y=args.y2),
                                 args.frame_id, args.tolerance)
        rospy.loginfo(result)
    except rospy.ROSInterruptException:
        pass
