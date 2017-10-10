#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger


class HMIDetermineResponseState(EventState):
    """
    State to determine the robot's response to some user input.

    -- target_time 	float 	Time which needs to have passed since the behavior started.

    ># input		string	Input to the robot

    #> output		string	Output to respond to the user
    #> grammar      string  Grammar for a counter-question
    #> target       string  Root element for the counter-question grammar

    <= continue 			Output selected

    """

    def __init__(self, target_time):
        super(HMIDetermineResponseState, self).__init__(outcomes=['succeeded'],
                                                        input_keys=['input'],
                                                        output_keys=['output'])

        # Store state parameter for later use.
        self._target_time = rospy.Duration(target_time)

        self._start_time = None

    def execute(self, userdata):
        user_input = userdata.input
        if user_input == "hey pepper":
            userdata.output = "Hi there! How are you?"
        elif user_input in ["great", "awesome", "pretty good", "alright", "super", "okay"]:
            userdata.output = "That's great!"
        elif user_input in ["not so good", "bad"]:
            userdata.output = "Oh dear... I'm sorry. Can I cheer you up?"
        else:
            userdata.output = "I don't know what to say."

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

