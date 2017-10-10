#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from hmi_msgs.msg import CastAction, CastGoal
from actionlib_msgs.msg import GoalStatus
import rospy


class HMICastState(EventState):
    """
    Implements a cast to the robot's user.

    This feedback can be given to the user using visual or audible feedback. For example using speech, a screen, lights,
    or motion.

    -- topic        string  The HMI topic to use
    -- timeout      float 	Time to wait for input before the cast times out (in seconds)

    ># message      string  The message to be cast to the user
    ># emotion		string	Optional emotion of the message

    <= timed_out			The cast timed out
    <= succeeded			The cast succeeded
    <= command_error        The command sent to the connected action server was invalid
    <= failed               The action failed for an unknown reason
    """

    def __init__(self, topic=None, timeout=10):
        super(HMICastState, self).__init__(outcomes=['timed_out', 'succeeded', 'command_error', 'failed'],
                                           input_keys=['message', 'emotion'])

        if not topic:
            rospy.logerr("Please set the topic to use for HMI casts.")
            raise Exception()
        self._topic = topic

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
            self._client.cancel_goal()

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
        goal.message = userdata.message
        if userdata.emotion:
            goal.emotion = userdata.emotion

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

