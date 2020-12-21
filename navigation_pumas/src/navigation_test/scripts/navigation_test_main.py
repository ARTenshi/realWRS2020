#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import tf2_ros
import math
import numpy as np
import random
import traceback

import hsrb_interface

from tf.transformations import quaternion_matrix, quaternion_from_euler, quaternion_from_matrix

from hsrb_interface import geometry
from geometry_msgs.msg import Vector3Stamped, Pose, PoseStamped, PointStamped
from geometry_msgs.msg import WrenchStamped, Twist

from common import speech, nav_tool_lib
from common.smach_states import *
from common import rospose_to_tmcpose

#from common import nav_module

from std_msgs.msg import Float32MultiArray, Bool

#from vizbox_bypass import BypassVizBox

##################################################



# Main
robot = hsrb_interface.Robot()

whole_body = robot.get("whole_body")
#omni_base = robot.get("omni_base")
omni_base=nav_tool_lib.nav_module("pumas")

gripper = robot.get('gripper')
tf_buffer = robot._get_tf2_buffer()

default_tts = speech.DefaultTTS()
console = speech.Console()

SAY = default_tts.say

whole_body.move_to_neutral()
rospy.loginfo('initializing...')
rospy.sleep(3)

pub = rospy.Publisher('/navigation/mvn_pln/get_close_xya', Float32MultiArray, queue_size=1)
#rospy.init_node('navigation_test', anonymous=True)

rate = rospy.Rate(10)

_get_destination = False

def callback_nav(data):
	global _get_destination
	_get_destination = data.data
	
def create_sm():

  sm = smach.StateMachine(outcomes=['success', 'failure'])

  with sm:
	##########
	#START: TASK INITIALISATION
	##########
	#Ask for the hand to be pushed to start
	smach.StateMachine.add('WAIT_HAND', WaitHandPushed(timeout=120.,
				say_fn=SAY,
				prompt_msg="Push the hand to start",
				success_msg=""),
				transitions={'success'	: 'TESTSTATE',
				'timeout'	: 'TESTSTATE',
				'failure'	: 'failure'})
	##########
	#END: TASK INITIALISATION
	##########

	##########
	#START: BASIC STATE FUNCTION
	##########
	@smach.cb_interface(outcomes=['success', 'failure'])
	def test_state_cb(self):
		try:
			#INSERT YOUR CODE HERE!!
			#FOR EXAMPLE:

			#move to origin
			#omni_base.go_abs(0.0, 0.0, 0.0)

			##PUMAS navigation node
			#pubMvnPlnGetCloseXYA = nh->advertise<std_msgs::Float32MultiArray>("/navigation/mvn_pln/get_close_xya", 1);

			#pub = rospy.Publisher('/navigation/mvn_pln/get_close_xya', Float32MultiArray, queue_size=1)
			#rospy.init_node('navigation_test', anonymous=True)
			#rate = rospy.Rate(10)

			#whole_body.move_to_go()
			#whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': -0.76})

			#msg = Float32MultiArray()
			#msg.layout.dim = []
			#msg.layout.data_offset = 0
			#[1.5741834850154284, -2.532385177708676, -1.0080257723968749]
			#msg.data = [0.73, 1.59, 0.88]
			#msg.data = [1.57, -2.53, -1.0] 

			#pub.publish(msg)
			#rate.sleep()
			#rospy.sleep(5.)

			#rospy.wait_for_service('/navigation/goal_reached')
			#get_goal_reached = rospy.ServiceProxy('/navigation/goal_reached', std_msgs/Bool)
			#while (True):
			#	rospy.Subscriber("/navigation/global_goal_reached", Bool, callback_nav)
				#rospy.loginfo("_get_destination: {}".format(_get_destination) )
				#rospy.Subscriber("/navigation/goal_reached", Bool, callback_nav)
			#	if _get_destination:
			#		break
				#rospy.spin()
			omni_base.go_abs(0.73,1.49,0.88)
			omni_base.go_rel(-.5,0,0)
			omni_base.go_abs(1.37, -2.53, -1.0, 0,"hsr")

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('TESTSTATE', smach.CBState(test_state_cb),
				transitions = {'success': 'success',
					       'failure': 'failure'})
	##########
	#END: BASIC STATE FUNCTION
	##########

  return sm

sm = create_sm()

###code for vizbox
#sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
#sis.start()
#viz_ins = BypassVizBox.initialize('test code challenge')
###

outcome = sm.execute()

if outcome == 'success':
	SAY('I finished the task.')
	#rospy.sleep(2)
	rospy.loginfo('I finished the task.')
else:
	SAY('Sorry, something happend. I did not finish the task.')
	rospy.signal_shutdown('Some error occured.')
