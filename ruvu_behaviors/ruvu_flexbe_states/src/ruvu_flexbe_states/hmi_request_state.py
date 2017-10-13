#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from hmi_msgs.msg import QueryAction, QueryGoal
from actionlib_msgs.msg import GoalStatus
import rospy


class HMIRequestState(EventState):
    """
    Implements a query to the robot's user. Input can be given using any HMI server such as a GUI or speech interaction.

    -- topic        string  The HMI topic to use
    -- timeout      float 	Time to wait for input before the request times out (in seconds)
    -- description  string  Description of the requested user input
    -- grammar		string	The grammar used for the request.
    -- target       string  The root element of the grammar tree

    #> sentence		string 	The sentence retrieved from the HMI server
    #> semantics	string	A json string containing the semantics of the sentence

    <= timed_out			The request timed out
    <= succeeded			The request resulted in input data
    <= command_error        The command sent to the connected action server was invalid
    <= failed               The action failed for an unknown reason
    """

    def __init__(self, topic=None, timeout=10, description="", grammar=None, target=None):
        super(HMIRequestState, self).__init__(outcomes=['timed_out', 'succeeded', 'command_error', 'failed'],
                                              output_keys=['sentence', 'semantics'])

        if not topic:
            rospy.logerr("Please set the topic to use for HMI requests.")
            raise Exception()
        self._topic = topic

        self._timeout = rospy.Duration(timeout)

        self._description = description

        if not grammar:
            rospy.logerr("Please set a grammar to use for HMI requests.")
            raise Exception()
        self._grammar = grammar

        if not target:
            rospy.logerr("Please set a target to use for HMI requests.")
            raise Exception()
        self._target = target

        self._client = ProxyActionClient({self._topic: QueryAction})

        # Initialize member variables
        self._error = False
        self._last_talker_id = ""
        self._last_feedback_time = None

    def execute(self, userdata):
        # Check if the client failed to send the goal.
        if self._error:
            return 'command_error'

        # Check if we got feedback. If so, reset the timer. If not, check if the timeout is exceeded yet
        if self._client.has_feedback(self._topic):
            rospy.loginfo("Received feedback")
            self._last_feedback_time = rospy.get_rostime()
        elif rospy.get_rostime() > self._last_feedback_time + self._timeout:
            # preempt action
            rospy.logdebug("Canceling goal")
            self._client.cancel(self._topic)

        # Check if the action has finished
        if self._client.has_result(self._topic):
            state = self._client.get_state(self._topic)

            if state == GoalStatus.SUCCEEDED:
                result = self._client.get_result(self._topic)
                userdata.sentence = result.sentence
                userdata.semantics = result.semantics
                self._last_talker_id = result.talker_id
                return 'succeeded'
            elif state == GoalStatus.PREEMPTED:
                return 'timed_out'
            else:
                return 'failed'

    def on_enter(self, userdata):
        # Make sure to reset all member variables since execution may enter this state more than once
        self._error = False
        self._last_talker_id = ""
        self._last_feedback_time = rospy.get_rostime()

        # Create the goal
        goal = QueryGoal()
        goal.description = self._description
        goal.grammar = self._grammar
        goal.target = self._target

        # Send the goal
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the HMIQuery goal:\n%s' % str(e))
            self._error = True

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')

