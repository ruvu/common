#!/usr/bin/env python  
import rospy
import math

import tf2_ros
import geometry_msgs.msg

x = 0
y = 0
yaw = 0
dt = 0.1
def callback(msg):
    # Update state
    global x, y, yaw
    x += msg.linear.x * dt * math.cos(yaw) - msg.linear.y * dt * math.sin(yaw)
    y += msg.linear.x * dt * math.sin(yaw) + msg.linear.y * dt * math.cos(yaw)
    yaw += msg.angular.z * dt

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "base_link"
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0

    t.transform.rotation.z = math.sin(yaw / 2.0)
    t.transform.rotation.w = math.cos(yaw / 2.0)

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('dummy_tf_broadcaster')
    t = rospy.Subscriber("cmd_vel", geometry_msgs.msg.Twist, callback, queue_size=1)
    rospy.spin()
