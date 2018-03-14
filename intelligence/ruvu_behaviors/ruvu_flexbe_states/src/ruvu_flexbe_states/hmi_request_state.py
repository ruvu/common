#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from hmi_msgs.msg import QueryAction, QueryGoal
from actionlib_msgs.msg import GoalStatus
import rospy
import json

from hmi import GrammarParser


def check_param(param, error_msg, condition=lambda x: x):
    if condition(param):
        return param
    else:
        rospy.logerr(error_msg)
        raise Exception(error_msg)


class HMIRequestState(EventState):
    """
    Implements a query to the robot's user. Input can be given using any HMI server such as a GUI or speech interaction.

    -- topic        string      The HMI topic to use
    -- timeout      float       Time to wait for input before the request times out (in seconds)
    -- description  string      Description of the requested user input
    -- grammar      text        The grammar used for the HMI request (as specified in hmi_msgs/Query)
    -- outcomes     [string]    List of outcomes of this state. Should be a subset of the grammar outcomes.
    -- num_examples int         Number of example responses to pass to HMI servers

    #> sentence     string      The sentence retrieved from the HMI server
    #> semantics    string      A json string containing the semantics of the sentence

    <= timed_out                The request timed out
    <= failed                   The action failed for an unknown reason
    """

    def __init__(self, topic=None, timeout=10, description="", grammar=None, target="T", outcomes=[], num_examples=10):
        self._grammar = check_param(grammar, "Please set the grammar to use for HMI requests.")

        self._target = check_param(target, "Please specify a valid target.")

        self._grammar_parser = GrammarParser.fromstring(self._grammar)
        self._grammar_parser.verify(target=self._target)

        # # Check if grammar outcomes are a subset of the specified outcomes
        # grammar_outcomes = set(_get_outcomes_from_grammar(self._grammar)).union({'timed_out', 'failed'})
        # check_param(set(outcomes), "Parameter 'outcomes' must be a subset of the grammar outcomes.",
        #             lambda x: x.issubset(grammar_outcomes))

        # Check if 'timed_out' and 'failed' are at least in the specified outcomes
        check_param({'timed_out', 'failed'}, "outcomes 'timed_out' and 'failed' must be in the paramter 'outcomes'",
                    lambda x: x.issubset(set(outcomes)))
        self._outcomes = outcomes

        super(HMIRequestState, self).__init__(outcomes=outcomes,
                                              output_keys=['sentence', 'semantics'])

        self._topic = check_param(topic, "Please set the topic to use for HMI requests.")

        check_param(timeout, "Timeout must be greater than zero", lambda x: x > 0)
        self._timeout = rospy.Duration(timeout)

        self._description = description

        self._client = ProxyActionClient({self._topic: QueryAction})

        # Initialize member variables
        self._error = False
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

                # If the server gave back semantics, use that, otherwise parse the sentence yourself
                if result.semantics:
                    semantics = json.loads(result.semantics)
                else:
                    semantics = self._grammar_parser.parse(self._target, result.sentence)

                # If neither the server, nor we were able to parse the sentence...
                if not isinstance(semantics, dict):
                    return 'failed'
                # If no outcome field is in the semantics...
                elif 'outcome' not in semantics:
                    return 'failed'
                # If the outcome is not one of the specified outcomes...
                elif semantics['outcome'] not in self._outcomes:
                    return 'failed'
                else:
                    return semantics['outcome']

            elif state == GoalStatus.PREEMPTED:
                return 'timed_out'
            else:
                return 'failed'

    def on_enter(self, userdata):
        # Make sure to reset all member variables since execution may enter this state more than once
        self._error = False
        self._last_feedback_time = rospy.get_rostime()

        # Create the goal
        goal = QueryGoal()
        goal.description = self._description
        goal.grammar = self._grammar
        goal.target = self._target
        goal.example_sentences = self._grammar_parser.get_random_sentences(self._target, num=10, max_num=1e5)

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
