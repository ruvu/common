# Copyright 2020 RUVU Robotics B.V.

import rospy
from actionlib_msgs.msg import GoalStatus
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from mbf_msgs.msg import GetPathAction, GetPathGoal


class GetPathState(EventState):
    """
    Request a path from a path planner.

    -- action       string                      The action topic namespace of the ExePath action
    -- exec_timeout int                         Number of seconds to wait for execution before preempting
    -- tolerance    float                       How many meters the planner can relax the constraint before failing
    -- planner      string                      Which planner to use

    ># target       geometry_msgs/PoseStamped   The calculated path

    #> path         nav_msgs/Path               The planned path to the target position

    <= succeeded                                Path successfully planned
    <= failed                                   Failed to plan a path
    <= preempted                                Planning took too long
    <= error                                    Failed to send the goal to the planner
    """

    def __init__(self, action, exec_timeout=None, tolerance=None, planner=None):
        super(GetPathState, self).__init__(
            outcomes=['succeeded', 'failed', 'preempted', 'error'], input_keys=['target'], output_keys=['path'])

        self._action = action
        self._client = ProxyActionClient({self._action: GetPathAction})

        if exec_timeout is not None:
            self._exec_timeout = rospy.Duration(exec_timeout)
        else:
            self._exec_timeout = None

        self._tolerance = tolerance
        self._planner = planner

        self._error = False
        self._start_time = rospy.get_rostime()

    def execute(self, userdata):
        # Check if the client failed to send the goal.
        if self._error:
            return 'error'

        state = self._client.get_state(self._action)
        if self._client.has_result(self._action):
            rospy.loginfo('%s: state is now %s' % (__name__, GoalStatus.to_string(state)))

            result = self._client.get_result(self._action)
            if result.message:
                Logger.logwarn(result.message)
            if state == GoalStatus.SUCCEEDED:
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
        goal = GetPathGoal(target_pose=userdata['target'], tolerance=self._tolerance, planner=self._planner)

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
