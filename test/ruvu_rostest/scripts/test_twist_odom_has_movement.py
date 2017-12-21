#!/usr/bin/env python
import unittest
import rostest
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3


class TestTwistOdomHasMovement(unittest.TestCase):

    def __init__(self, *args):
        unittest.TestCase.__init__(self, *args)
        rospy.init_node('test_twist_odom_has_movement')

    def odom_callback(self, msg):
        self._odom_msg = msg

    def test_twist_odom_has_movement(self):
        timeout = rospy.get_param("~timeout", 10.0)
        twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Wait for connections
        while twist_pub.get_num_connections() == 0:
            rospy.Rate(10).sleep()

        start = rospy.Time.now()
        while not rospy.is_shutdown() and rospy.Time.now() - start < rospy.Duration(timeout):
            twist_pub.publish(Twist(linear=Vector3(x=0.2, y=0.2)))
            rospy.loginfo("Publishing ...")
            rospy.Rate(10).sleep()
        twist_pub.unregister()

        self._odom_msg = None
        self._subscriber = rospy.Subscriber('odom', Odometry, self.odom_callback)
        while not self._odom_msg:
            rospy.sleep(.1)
        self._subscriber.unregister()

        # Check if we have moved
        self.assertNotEqual(0, self._odom_msg.pose.pose.position.x)
        self.assertNotEqual(0, self._odom_msg.pose.pose.position.y)


PKG = 'ruvu_rostest'
NAME = 'test_twist_odom_has_movement'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestTwistOdomHasMovement)