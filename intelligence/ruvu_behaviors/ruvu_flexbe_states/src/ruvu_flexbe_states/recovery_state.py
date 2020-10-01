#!/usr/bin/env python

# Copyright 2020 RUVU Robotics B.V.

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from mbf_msgs.msg import RecoveryAction, RecoveryGoal
from actionlib_msgs.msg import GoalStatus
import rospy


class RecoveryState(EventState):
    """
    Execute a recovery behavior.

    -- action       string          The action topic namespace of the GetPath action
    -- exec_timeout int             Number of seconds to wait for execution before preempting

    ># path         nav_msgs/Path   The sentence retrieved from the HMI server

    <= succeeded                    Goal reached
    <= failed                       Failed to reach the goal
    <= preempted                    Reaching the goal took too long
    <= error                        Error sending the goal
    """

    def __init__(self, action, exec_timeout=None):
        super(RecoveryState, self).__init__(outcomes=['succeeded', 'failed', 'preempted', 'error'],
                                            input_keys=['error', 'clear_costmap_flag'],
                                            output_keys=['error_status', 'clear_costmap_flag'])

        if exec_timeout:
            self._exec_timeout = rospy.Duration(exec_timeout)
        else:
            self._exec_timeout = None

        self._action = action
        self._client = ProxyActionClient({self._action: RecoveryAction})

        self._error = False
        self._start_time = rospy.get_rostime()

    def execute(self, userdata):
        # Check if the client failed to send the goal.
        if self._error:
            return 'error'

        state = self._client.get_state(self._action)

        if self._client.has_result(self._action):
            if state == GoalStatus.SUCCEEDED:
                return 'succeeded'

            elif state == GoalStatus.PREEMPTED:
                return 'preempted'
            else:
                return 'failed'

        if self._exec_timeout and state != GoalStatus.PREEMPTING:
            if rospy.get_rostime() - self._start_time > self._exec_timeout:
                self._client.cancel(self._action)

    def on_enter(self, userdata):
        # Make sure to reset all member variables since execution may enter this state more than once
        self._error = False
        self._start_time = rospy.get_rostime()

        # Create the goal
        goal = RecoveryGoal()

        if userdata.clear_costmap_flag:
            goal.behavior = 'clear_costmap'
            userdata.clear_costmap_flag = False
        else:
            goal.behavior = 'straf_recovery'
            userdata.clear_costmap_flag = True

        # Send the goal
        try:
            self._client.send_goal(self._action, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the GetPathAction goal:\n%s' % str(e))
            self._error = True

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        if not self._client.has_result(self._action):
            self._client.cancel(self._action)
            Logger.loginfo('Cancelled active GetPathAction goal.')
