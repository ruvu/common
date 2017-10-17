#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_facetracking')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ruvu_flexbe_states.face_tracking_state import FaceTrackingState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Oct 17 2017
@author: Rokus Ottervanger
'''
class FaceTrackingSM(Behavior):
	'''
	Face tracking
	'''


	def __init__(self):
		super(FaceTrackingSM, self).__init__()
		self.name = 'FaceTracking'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:550 y:280, x:555 y:148
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:174 y:143
			OperatableStateMachine.add('face_tracking',
										FaceTrackingState(head_topic='follow_joint_trajectory', faces_topic='faces', timeout=5, pan_frame_id='torso', tilt_frame_id='Neck', head_frame_id='Head'),
										transitions={'preempted': 'finished', 'error': 'failed'},
										autonomy={'preempted': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
