#!/usr/bin/env python
import rospy
import tf
import math

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached, ProxyTransformListener

from people_perception_msgs.msg import FaceDetections

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PointStamped


class FaceTrackingState(EventState):
    """
    FaceTrackingState This state sends joint goals for the neck joints to track the closest face provided.

    <= preempted            The state was requested to stop and succesfully stopped
    <= error                An unknown error occurred

    """

    def __init__(self, head_topic='follow_joint_trajectory', faces_topic='faces', timeout=1, pan_frame_id='torso', tilt_frame_id='neck', head_frame_id='head'):
        # See example_state.py for basic explanations.
        super(FaceTrackingState, self).__init__(outcomes=['preempted', 'error'])

        self._head_topic = head_topic
        self._head_client = ProxyActionClient({self._head_topic: FollowJointTrajectoryAction})

        self._faces_topic = faces_topic
        self._faces_sub = ProxySubscriberCached({self._faces_topic: FaceDetections})

        tf_listener_proxy = ProxyTransformListener()
        self._tf = tf_listener_proxy.listener()

        self._timeout = rospy.Duration(timeout)

        self._pan_frame_id = pan_frame_id
        self._tilt_frame_id = tilt_frame_id
        self._head_frame_id = head_frame_id

        self._last_closest_face_point_stamped = None

        # It may happen that the action client fails to send the action goal.
        self._error = False

    def _get_pan_tilt(self):
        pan, tilt = 0, 0

        if not self._last_closest_face_point_stamped:
            return pan, tilt

        try:
            target_pan = self._tf.transformPoint(self._pan_frame_id, self._last_closest_face_point_stamped)
        except tf.Exception as e:
            rospy.logwarn(e)
            return pan, tilt

        try:
            target_tilt = self._tf.transformPoint(self._tilt_frame_id, self._last_closest_face_point_stamped)
        except tf.Exception as e:
            rospy.logwarn(e)
            return pan, tilt

        try:
            transform = self._tf.lookupTransform(self._pan_frame_id, self._tilt_frame_id, rospy.Time())
            pan_to_tilt = transform[0][0]  # Get X of the translation
        except tf.Exception as e:
            rospy.logwarn(e)
            return pan, tilt

        # try:
        #     transform = self._tf.lookupTransform(self._tilt_frame_id, self._head_frame_id, rospy.Time())
        #     tilt_to_head_vert = transform[0][2]  # Get Z of the translation
        # except tf.Exception as e:
        #     rospy.logwarn(e)
        #     return pan, tilt

        pan = math.atan2(target_pan.point.y, target_pan.point.x + pan_to_tilt)

        tilt = -math.atan2(target_tilt.point.z, target_tilt.point.x)

        rospy.loginfo("(pan, tilt) = %f, %f" % (pan, tilt))

        return pan, tilt

    def _update_closest_face(self):
        if self._faces_sub.has_msg(self._faces_topic):
            msg = self._faces_sub.get_last_msg(self._faces_topic)
            closest_distance = 1e9
            point = None
            for detection in msg.detections:
                if detection.position.z < closest_distance:
                    closest_distance = detection.position.z
                    point = PointStamped(
                        header=msg.header,
                        point=detection.position
                    )
            if point :
                self._last_closest_face_point_stamped = point

            self._faces_sub.remove_last_msg(self._faces_topic)

        if self._last_closest_face_point_stamped and rospy.get_rostime() - \
                self._last_closest_face_point_stamped.header.stamp > self._timeout:
            self._last_closest_face_point_stamped = None

    def execute(self, userdata):
        # Check if the client failed to send the goal.
        if self._error:
            return 'error'

        self._update_closest_face()

        pan, tilt = self._get_pan_tilt()

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['HeadYaw', 'HeadPitch']
        point = JointTrajectoryPoint()
        point.positions += [pan, tilt]
        point.velocities += [0, 0]
        point.accelerations += [0, 0]
        point.effort += [0.05, 0.05]
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)

        # Send the goal.
        try:
            self._head_client.send_goal(self._head_topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the head command:\n%s' % str(e))
            self._error = True

    def on_enter(self, userdata):
        self._last_closest_face_point_stamped = None
        self._error = False

    def on_exit(self, userdata):
        if not self._head_client.has_result(self._head_topic):
            self._head_client.cancel(self._head_topic)
            Logger.loginfo('Cancelled active action goal.')

