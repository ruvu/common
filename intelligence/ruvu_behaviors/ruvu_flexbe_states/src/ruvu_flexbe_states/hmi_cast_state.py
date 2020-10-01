#!/usr/bin/env python

# Copyright 2020 RUVU Robotics B.V.

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from hmi_msgs.msg import CastAction, CastGoal
from actionlib_msgs.msg import GoalStatus
import rospy


def check_param(param, error_msg, condition=lambda x: x):
    if condition(param):
        return param
    else:
        rospy.logerr(error_msg)
        raise Exception(error_msg)


class HMICastState(EventState):
    """
    Implements a cast to the robot's user.

    This feedback can be given to the user using visual or audible feedback. For example using speech, a screen, lights,
    or motion.

    -- topic        string  The HMI topic to use
    -- timeout      float 	Time to wait for input before the cast times out (in seconds)
    -- message      string  The message to be cast to the user
    -- emotion		string	Optional emotion of the message

    <= succeeded			The cast succeeded
    <= failed               The cast failed for an unknown reason
    <= timed_out            The cast timed out
    """

    def __init__(self, topic, message, timeout=10, emotion=0):
        super(HMICastState, self).__init__(outcomes=['succeeded', 'timed_out', 'failed'])

        self._topic = check_param(topic, "Please set the topic to use for HMI casts.")

        self._message = check_param(message, "Message cannot be empty.")
        self._emotion = emotion

        timeout = check_param(timeout, "Timeout should be greater than zero", lambda x: x > 0)
        self._timeout = rospy.Duration(timeout)

        self._client = ProxyActionClient({self._topic: CastAction})

        # Initialize member variables
        self._error = False
        self._enter_time = rospy.get_rostime()

    def execute(self, userdata):
        # Check if the client failed to send the goal.
        if self._error:
            return 'command_error'

        if rospy.get_rostime() > self._enter_time + self._timeout:
            # preempt action
            rospy.logdebug("Canceling goal")
            self._client.cancel(self._topic)

        # Check if the action has finished
        if self._client.has_result(self._topic):
            state = self._client.get_state(self._topic)

            if state == GoalStatus.SUCCEEDED:
                return 'succeeded'
            elif state == GoalStatus.PREEMPTED:
                return 'timed_out'
            else:
                return 'failed'

    def on_enter(self, userdata):
        # Make sure to reset all member variables since execution may enter this state more than once
        self._error = False
        self._enter_time = rospy.get_rostime()

        # Create the goal
        goal = CastGoal()
        goal.message = self._message
        goal.emotion = self._emotion

        # Send the goal
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the HMICast goal:\n%s' % str(e))
            self._error = True

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active HMI Cast action goal.')
