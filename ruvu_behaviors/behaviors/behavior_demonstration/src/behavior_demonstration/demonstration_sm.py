#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_demonstration')
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
class DemonstrationSM(Behavior):
	'''
	Simple demonstration behavior
	'''


	def __init__(self):
		super(DemonstrationSM, self).__init__()
		self.name = 'Demonstration'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		grammar_prefix = """#BNF+EMV1.1; !grammar "grammar"; !start <start>; !pronounce I PRONAS "i"; !pronounce I PRONAS "i";"""
		tts_topic = "text_to_speech"
		speech_recognition_topic = "speech_recognition"
		# x:903 y:265, x:912 y:47
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.emotion = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:161 y:49
			OperatableStateMachine.add('wait_for_user_input',
										HMIRequestState(topic=speech_recognition_topic, timeout=10, description="Say 'Hey Pepper'", grammar=grammar_prefix + """ <start> : hey pepper;""", target="<start>"),
										transitions={'timed_out': 'failed', 'succeeded': 'give_response', 'command_error': 'failed', 'failed': 'failed'},
										autonomy={'timed_out': Autonomy.Off, 'succeeded': Autonomy.Off, 'command_error': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'sentence': 'sentence', 'semantics': 'semantics'})

			# x:634 y:143
			OperatableStateMachine.add('give_response',
										HMICastState(topic=tts_topic, timeout=20),
										transitions={'timed_out': 'failed', 'succeeded': 'finished', 'command_error': 'failed', 'failed': 'failed'},
										autonomy={'timed_out': Autonomy.Off, 'succeeded': Autonomy.Off, 'command_error': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'message': 'sentence', 'emotion': 'emotion'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
