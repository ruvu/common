#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_conversation')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ruvu_flexbe_states.hmi_request_state import HMIRequestState
from ruvu_flexbe_states.hmi_cast_state import HMICastState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Oct 10 2017
@author: Rokus Ottervanger
'''
class ConversationSM(Behavior):
	'''
	Simple conversation behavior
	'''


	def __init__(self):
		super(ConversationSM, self).__init__()
		self.name = 'Conversation'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		tts_topic = "hmi/cast"
		speech_recognition_topic = "hmi/query"
		# x:1455 y:758, x:1380 y:40
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:198 y:70
			OperatableStateMachine.add('ask_service',
										HMICastState(topic=tts_topic, message="What can I do for you?", timeout=3, emotion=0),
										transitions={'succeeded': 'query_service', 'timed_out': 'ask_service', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off})

			# x:626 y:191
			OperatableStateMachine.add('navigate_response',
										HMICastState(topic=tts_topic, message="You still need to teach me to navigate properly.", timeout=10, emotion=0),
										transitions={'succeeded': 'finished', 'timed_out': 'failed', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off})

			# x:626 y:124
			OperatableStateMachine.add('grab_response',
										HMICastState(topic=tts_topic, message="Don't you see I cannot grasp anything?", timeout=10, emotion=0),
										transitions={'succeeded': 'finished', 'timed_out': 'failed', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off})

			# x:624 y:259
			OperatableStateMachine.add('say_response',
										HMICastState(topic=tts_topic, message="I don't know what to say", timeout=10, emotion=0),
										transitions={'succeeded': 'finished', 'timed_out': 'failed', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off})

			# x:627 y:371
			OperatableStateMachine.add('answer_question',
										HMICastState(topic=tts_topic, message="I can't answer questions yet", timeout=10, emotion=0),
										transitions={'succeeded': 'finished', 'timed_out': 'failed', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off})

			# x:263 y:259
			OperatableStateMachine.add('query_service',
										HMIRequestState(topic=speech_recognition_topic, timeout=10, description="What can I do for you?", grammar=T[{actions : <A>}] -> VP[A]; T[{actions : <A1, A2>}] -> VP[A1] and VP[A2]; V_GRAB -> grab | grasp | fetch; VP[{outcome: "grab", entity: E}] -> V_GRAB the ENTITIES[E]; V_NAV -> navigate to | go to; VP[{outcome: "navigate-to", entity: LOC}] -> V_NAV the LOCATIONS[LOC]; LOCATIONS[{id: "livingroom"}] -> living room; LOCATIONS[{id: "kitchen"}] -> kitchen; LOCATIONS[{id: "hallway"}] -> hallway; LOCATIONS[{id: "corridor"}] -> corridor; ENTITIES[{id: "coke"}] -> coke; ENTITIES[{id: "fanta"}] -> fanta; ENTITIES[{id: "oj"}] -> orange juice; PRONOWN -> a | the | his | her; VP[{"outcome": "answer-question"}] -> answer PRONOWN question; V_PLACE -> put | place; VP[{"outcome": "bring", "to" : L}] -> V_PLACE SMALL_OBJECT_SPEC[X] on the LOCATIONS[L]; VP[{"outcome": "bring", "to" : L}] -> V_PLACE SMALL_OBJECT_SPEC[X] in the LOCATIONS[L]; SMALL_OBJECT_SPEC[{ "type" : X }] -> DET SMALL_OBJECT[X]; SMALL_OBJECT[{id: "coke"}] -> coke; DET -> the | a; V_SAY -> tell | say | speak; VP[{"outcome": "say", "sentence": "ROBOT_NAME"}] -> V_SAY your name; VP[{"outcome": "say", "sentence": "TIME"}] -> V_SAY the time | V_SAY what time it is | V_SAY what time is it; VP[{"outcome": "say", "sentence": "my team is tech united"}] -> V_SAY the name of your team; VP[{"outcome": "say", "sentence": "DAY_OF_MONTH"}] -> V_SAY the day of the month; VP[{"outcome": "say", "sentence": "DAY_OF_WEEK"}] -> V_SAY the day of the week; VP[{"outcome": "say", "sentence": "TODAY"}] -> V_SAY what day is today | V_SAY me what day it is | V_SAY the date; VP[{"outcome": "say", "sentence": "TOMORROW"}] -> V_SAY what day is tomorrow; V_BRING -> give | bring | hand | deliver | take | carry | transport; VP[{"outcome": "bring", "entity" : X, "to" : Y}] -> V_BRING DET ENTITIES[X] to DET LOCATIONS[Y], target="T", outcomes=['grab', 'navigate-to', 'answer-question', 'bring', 'say', 'timed_out', 'failed'], num_examples=2),
										transitions={'grab': 'grab_response', 'navigate-to': 'navigate_response', 'say': 'say_response', 'answer-question': 'answer_question', 'bring': 'failed', 'timed_out': 'failed', 'failed': 'failed'},
										autonomy={'grab': Autonomy.Off, 'navigate-to': Autonomy.Off, 'say': Autonomy.Off, 'answer-question': Autonomy.Off, 'bring': Autonomy.Off, 'timed_out': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'sentence': 'sentence', 'semantics': 'semantics'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
