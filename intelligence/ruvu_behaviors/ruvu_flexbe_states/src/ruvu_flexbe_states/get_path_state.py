#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from move_base_flex_msgs.msg import GetPathAction, GetPathGoal
from actionlib_msgs.msg import GoalStatus
import rospy


class GetPathState(EventState):
    """
    Request a path from a path planner.

    -- action       string                      The action topic namespace of the GetPath action
    -- exec_timeout int                         Number of seconds to wait for execution before preempting

    ># target       geometry_msgs/PoseStamped   The sentence retrieved from the HMI server

    #> path         nav_msgs/Path               The planned path to the target position

    <= succeeded                                Path successfully planned
    <= failed                                   Failed to plan a path
    <= preempted                                Planning took too long
    <= error                                    Failed to send the goal to the planner
    """

    def __init__(self, action, exec_timeout=None):
        super(GetPathState, self).__init__(outcomes=['succeeded', 'failed', 'preempted', 'error'],
                                           input_keys=['target'],
                                           output_keys=['path'])

        if exec_timeout is not None:
            self._exec_timeout = rospy.Duration(exec_timeout)
        else:
            self._exec_timeout = None

        self._action = action
        self._client = ProxyActionClient({self._action: GetPathAction})

        self._error = False
        self._start_time = rospy.get_rostime()

    def execute(self, userdata):
        # Check if the client failed to send the goal.
        if self._error:
            return 'error'

        state = self._client.get_state(self._action)

        if self._client.has_result(self._action):
            if state == GoalStatus.SUCCEEDED:
                result = self._client.get_result(self._action)
                userdata['path'] = result.path
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
        goal = GetPathGoal(target_pose=userdata['target_pose_stamped'])

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

