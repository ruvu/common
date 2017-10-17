#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_chatter')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ruvu_flexbe_states.hmi_request_state import HMIRequestState
from ruvu_flexbe_states.hmi_cast_state import HMICastState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Oct 13 2017
@author: Rokus Ottervanger
'''
class ChatterSM(Behavior):
	'''
	Bla bla bla
	'''


	def __init__(self):
		super(ChatterSM, self).__init__()
		self.name = 'Chatter'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		hmi_cast_topic = "hmi/cast"
		hmi_query_topic = "hmi/query"
		# x:1310 y:587, x:1305 y:57
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:202 y:88
			OperatableStateMachine.add('main_request_state',
										HMIRequestState(topic=hmi_query_topic, timeout=10, description="Greet me to trigger me.", grammar="T[{'outcome': 'triggered'}] -> GREETING ROBOT_NAME; GREETING -> hi | hey | hi there | hey there | hello | hello there; ROBOT_NAME -> pepper", target="T", outcomes=['triggered', 'timed_out', 'failed'], num_examples=10),
										transitions={'triggered': 'whats_up', 'timed_out': 'main_request_state', 'failed': 'failed'},
										autonomy={'triggered': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'sentence': 'sentence', 'semantics': 'semantics'})

			# x:408 y:121
			OperatableStateMachine.add('whats_up',
										HMICastState(topic=hmi_cast_topic, message="Hi there!", timeout=10, emotion=0),
										transitions={'succeeded': 'request_2', 'timed_out': 'main_request_state', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off})

			# x:519 y:294
			OperatableStateMachine.add('request_2',
										HMIRequestState(topic=hmi_query_topic, timeout=10, description="Ask me something.", grammar="T[{'outcome': 'say_name'}] -> what is your name | who are you | tell me your name; T[{'outcome': 'say_robot'}] -> what are you; T[{'outcome': 'say_length'}] -> how tall are you | what is your size; T[{'outcome': 'say_age'}] -> how old are you | what is your age; T[{'outcome': 'talk_movies'}] -> lets talk about movies | tell me about movies; T[{'outcome': 'talk_weather'}] -> lets talk about the weather | tell me about the weather; T[{'outcome': 'exit'}] -> exit", target="T", outcomes=['say_name', 'say_robot', 'say_length', 'say_age', 'talk_movies', 'talk_weather', 'timed_out', 'failed', 'exit'], num_examples=10),
										transitions={'say_name': 'say_name', 'say_robot': 'say_robot', 'say_length': 'say_length', 'say_age': 'say_age', 'talk_movies': 'talk_movies', 'talk_weather': 'talk_weather', 'timed_out': 'main_request_state', 'failed': 'failed', 'exit': 'finished'},
										autonomy={'say_name': Autonomy.Off, 'say_robot': Autonomy.Off, 'say_length': Autonomy.Off, 'say_age': Autonomy.Off, 'talk_movies': Autonomy.Off, 'talk_weather': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off, 'exit': Autonomy.Off},
										remapping={'sentence': 'sentence', 'semantics': 'semantics'})

			# x:906 y:113
			OperatableStateMachine.add('say_name',
										HMICastState(topic=hmi_cast_topic, message="I am Pepper", timeout=10, emotion=0),
										transitions={'succeeded': 'say_robot', 'timed_out': 'failed', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1050 y:162
			OperatableStateMachine.add('say_robot',
										HMICastState(topic=hmi_cast_topic, message="I am a robot", timeout=10, emotion=0),
										transitions={'succeeded': 'request_2', 'timed_out': 'failed', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1046 y:255
			OperatableStateMachine.add('say_length',
										HMICastState(topic=hmi_cast_topic, message="I am 3 meters tall", timeout=10, emotion=0),
										transitions={'succeeded': 'request_2', 'timed_out': 'failed', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1045 y:315
			OperatableStateMachine.add('say_age',
										HMICastState(topic=hmi_cast_topic, message="Depends on what you mean, but I was released in 2015", timeout=10, emotion=0),
										transitions={'succeeded': 'request_2', 'timed_out': 'failed', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1035 y:387
			OperatableStateMachine.add('talk_movies',
										HMICastState(topic=hmi_cast_topic, message="I don't know any movies yet", timeout=10, emotion=0),
										transitions={'succeeded': 'request_2', 'timed_out': 'failed', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1029 y:458
			OperatableStateMachine.add('talk_weather',
										HMICastState(topic=hmi_cast_topic, message="I have never been outside, so I don't know what weather means", timeout=10, emotion=0),
										transitions={'succeeded': 'request_2', 'timed_out': 'failed', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
