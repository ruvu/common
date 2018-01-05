#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class OdomToPoseNode():
    def __init__(self):
        self.odom_sub = rospy.Subscriber('odom', Odometry, callback=self.callback)
        self.publisher = rospy.Publisher('pose', PoseWithCovarianceStamped, queue_size=10)

    def callback(self, msg):
        pose = PoseWithCovarianceStamped()
        pose.header = msg.header
        pose.pose = msg.pose

        self.publisher.publish(pose)


if __name__ == '__main__':
    rospy.init_node('odom_to_pose')

    OdomToPoseNode()
    rospy.spin()
