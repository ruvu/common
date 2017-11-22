#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_simple_navigation')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.subscriber_state import SubscriberState
from ruvu_flexbe_states.get_path_state import GetPathState
from ruvu_flexbe_states.execute_path_state import ExecutePathState
from ruvu_flexbe_states.recovery_state import RecoveryState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Nov 21 2017
@author: Rokus Ottervanger
'''
class Simple_NavigationSM(Behavior):
	'''
	Simple navigation state machine using Move Base Flex
	'''


	def __init__(self):
		super(Simple_NavigationSM, self).__init__()
		self.name = 'Simple_Navigation'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:782 y:73, x:79 y:395
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.error = None
		_state_machine.userdata.clear_costmap_flag = False

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:68 y:77
			OperatableStateMachine.add('subscriber_state',
										SubscriberState(topic='/move_base_simple/goal', blocking=True, clear=False),
										transitions={'received': 'get_path', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'target'})

			# x:290 y:76
			OperatableStateMachine.add('get_path',
										GetPathState(action='move_base_flex/get_path', exec_timeout=3),
										transitions={'succeeded': 'execute_path', 'failed': 'failed', 'preempted': 'failed', 'error': 'failed'},
										autonomy={'succeeded': Autonomy.High, 'failed': Autonomy.High, 'preempted': Autonomy.High, 'error': Autonomy.High},
										remapping={'target': 'target', 'path': 'path'})

			# x:547 y:72
			OperatableStateMachine.add('execute_path',
										ExecutePathState(action='move_base_flex/exe_path', exec_timeout=None),
										transitions={'succeeded': 'finished', 'failed': 'recover', 'preempted': 'failed', 'error': 'failed'},
										autonomy={'succeeded': Autonomy.Low, 'failed': Autonomy.Low, 'preempted': Autonomy.Low, 'error': Autonomy.Low},
										remapping={'path': 'path'})

			# x:487 y:221
			OperatableStateMachine.add('recover',
										RecoveryState(action='move_base_flex/recovery', exec_timeout=None),
										transitions={'succeeded': 'get_path', 'failed': 'failed', 'preempted': 'failed', 'error': 'failed'},
										autonomy={'succeeded': Autonomy.Low, 'failed': Autonomy.Low, 'preempted': Autonomy.Low, 'error': Autonomy.Low},
										remapping={'error': 'error', 'clear_costmap_flag': 'clear_costmap_flag', 'error_status': 'error_status'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
