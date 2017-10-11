#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from dialogflow_msgs.msg import TextRequestAction, TextRequestGoal
from actionlib_msgs.msg import GoalStatus


class HMIDetermineResponseState(EventState):
    """
    State to determine the robot's response to user input.

    -- topic        string  Topic for communication with the action server for forming a response.

    ># text 		string	Input to the robot

    #> semantics	string	Output to respond to the user
    #> response     string  Grammar for a counter-question

    <= succeeded    		Output selected
    <= failed               Some error occurred

    """

    def __init__(self, topic=None):
        super(HMIDetermineResponseState, self).__init__(outcomes=['succeeded', 'failed'],
                                                        input_keys=['text'],
                                                        output_keys=['semantics', 'response'])

        if not topic:
            rospy.logerr("Please set the topic to use for communication with the conversation engine.")
            raise Exception()
        self._topic = topic

        self._client = ProxyActionClient({self._topic: TextRequestAction})
        self._error = False

    def execute(self, userdata):
        if self._error:
            return 'failed'

        # Check if the action has finished
        if self._client.has_result(self._topic):
            state = self._client.get_state(self._topic)

            if state == GoalStatus.SUCCEEDED:
                result = self._client.get_result(self._topic)
                userdata.response = result.response
                userdata.semantics = result.semantics
                return 'succeeded'
            else:
                return 'failed'

        if self._error:
            return 'command_error'

    def on_enter(self, userdata):
        if not userdata.text:
            self._error = True

        goal = TextRequestGoal()
        goal.text = userdata.text

        # Send the goal
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the conversation goal:\n%s' % str(e))
            self._error = True

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active conversation action goal.')

    def on_start(self):
        pass

    def on_stop(self):
        pass

