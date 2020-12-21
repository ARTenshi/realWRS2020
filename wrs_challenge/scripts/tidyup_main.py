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
import time
import actionlib

import hsrb_interface
from hsrb_interface import geometry

import moveit_commander

from tf.transformations import quaternion_matrix, quaternion_from_euler, quaternion_from_matrix

from geometry_msgs.msg import Vector3Stamped, Pose, PoseStamped, PointStamped
from geometry_msgs.msg import WrenchStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from common import speech
from common.smach_states import *
from common import rospose_to_tmcpose

from navigation_tamagoya.nav_tool_lib import nav_module
from interface.oss_vision import FindEdge, DetectObjectOnPlane, DetectObjectOnVerticalPlane, DetectGraspObjectOnFloor
from interface.modules import FunctionParams
##################################################

_DISTANCE = 0.80
_ORIENTATION = 0.0001
_LOCATION = 0
_CENTROID = [500, 300]
_ALIGN = True
_RIGHTHANDLE = True
_OPENDRAWER = True
_TIMETASK1 = 800 #800 #600

_NUMOBJECTSSHELF = 1 #3
GRASP_THRESHOLD = -0.865

class myRobot:
	whole_body = []
	head = []
	arm = []
	gripper = []
	omni_base = []
	cmd_vel = []
	navclient = []
	

# Main
robot = myRobot()#hsrb_interface.Robot()

robot.whole_body = moveit_commander.MoveGroupCommander("whole_body_weighted")
robot.head = moveit_commander.MoveGroupCommander("head")
robot.arm = moveit_commander.MoveGroupCommander('arm')
robot.gripper = moveit_commander.MoveGroupCommander("gripper")
robot.omni_base = nav_module("pumas")
robot.cmd_vel = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
robot.navclient = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

#default_tts = speech.DefaultTTS()
#console = speech.Console()

#SAY = default_tts.say
SAY = None

#whole_body.move_to_neutral()
robot.arm.stop()
robot.arm.set_named_target('neutral')
robot.arm.go(wait=True)
robot.arm.stop()

rospy.loginfo('initializing...')
rospy.sleep(3)

def create_sm():

  sm = smach.StateMachine(outcomes=['success', 'failure'])

  # save start time
  sm.userdata.start_time = time.time()

  # time limit in seconds
  sm.userdata.stop_time = _TIMETASK1 #800 #600

  # robot model
  sm.userdata.hsr = 'sim'

  sm.userdata.robot_info = RobotInfo()
  sm.userdata.task = "tidy up"

  sm.userdata.robot_info.make_obj_list(sm.userdata.task)
  sm.userdata.robot_info.make_location_list(sm.userdata.task)

  sm.userdata.obj_plane = -1.0
  sm.userdata.front_grasping = True

  sm.userdata.obj_name = "unknown"
  sm.userdata.search_pt = _LOCATION

  sm.userdata.location_name = "office"

  sm.userdata.fun_params = FunctionParams()

  sm.userdata.timetransition = False

  with sm:

	##########
	#START: TASK INITIALISATION
	##########
	#Ask for the hand to be pushed to start
	smach.StateMachine.add('WAIT_HAND', WaitHandPushed(timeout=1.,
				say_fn=SAY,
				prompt_msg="Push the hand to start",
				success_msg=""),
				transitions={'success'	: 'STARTTRIAL', #'STARTTRIAL', #OPENDRAWER #MOVE2LIVING1
				'timeout'	: 'STARTTRIAL', #'STARTTRIAL', #OPENDRAWER #MOVE2LIVING1
				'failure'	: 'failure'})
	##########
	#END: TASK INITIALISATION
	##########

	##########
	#START: BASIC OPEN DRAWER FUNCTION
	##########
	#Move to first drawer
	@smach.cb_interface(outcomes=['success', 'failure'])
	def open_drawer_cb(userdata):
		try:
			if(_OPENDRAWER):
				rospy.loginfo('I will try to open the drawer.')

				return 'success'
			else:
				return 'failure'
		except:
			return 'failure'

	smach.StateMachine.add('OPENDRAWER', smach.CBState(open_drawer_cb),
				transitions = {'success': 'MOVE2DRAWERA',
					       'failure': 'STARTTRIAL'})

	@smach.cb_interface(outcomes=['success', 'failure'],
			    input_keys=['fun_params', 'robot_info'],
			    output_keys=['fun_params'])
	def move_to_drawera_cb(userdata):
		try:
			loc_key = "drawer bottom close"
			_pos = userdata.robot_info.location_list[loc_key.upper()].position
			rospy.loginfo("Opening {} at {}, {}, {}".format(loc_key, _pos.x, _pos.y, _pos.yaw))

			userdata.fun_params.robot_pose.delay = 1.0
			userdata.fun_params.robot_pose.tilt = -0.96
			userdata.fun_params.robot_pose.distance = _DISTANCE
			userdata.fun_params.robot_pose.height_diff = 0.25
			userdata.fun_params.robot_pose.wrist_roll = 0.0001#-1.57#0.0
			userdata.fun_params.robot_pose.in_use = True

			userdata.fun_params.edges_bb.position = _DISTANCE
			userdata.fun_params.edges_bb.orientation = 0.0001
			userdata.fun_params.edges_bb.maxx = 0.30
			userdata.fun_params.edges_bb.miny = 0.15
			userdata.fun_params.edges_bb.maxy = 0.35
			userdata.fun_params.edges_bb.maxz = 1.50
			userdata.fun_params.edges_bb.in_use = True

			userdata.fun_params.objects_bb.depth_min = 0.40
			userdata.fun_params.objects_bb.depth_max = 1.20
			userdata.fun_params.objects_bb.width = 0.30
			userdata.fun_params.objects_bb.height_min = _pos.z - 0.25
			userdata.fun_params.objects_bb.height_max = _pos.z + 0.025
			userdata.fun_params.objects_bb.in_use = True
			
			robot.omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)

			return 'success'

		except:
			return 'failure'

	smach.StateMachine.add('MOVE2DRAWERA', smach.CBState(move_to_drawera_cb),
				transitions = {'success': 'ALIGN2DRAWERA',
					       'failure': 'OPENDRAWERERROR'})

	smach.StateMachine.add('ALIGN2DRAWERA', FindEdge(robot, align = True),
                               transitions = {'success': 'DETECTHANDLEA',
                                              'unknown': 'DETECTHANDLEA',
                                              'timeout': 'OPENDRAWERERROR',
                                              'failure': 'OPENDRAWERERROR'})

	#Bottom drawer
	smach.StateMachine.add('DETECTHANDLEA', DetectObjectOnVerticalPlane(robot, target = -0.05),
                               transitions = {'success': 'OPENDRAWERA',
                                              'unknown': 'OPENDRAWERERROR',
                                              'timeout': 'OPENDRAWERERROR',
                                              'failure': 'OPENDRAWERERROR'})

	#Performs all the hard-coded steps to grasp the object
	@smach.cb_interface(outcomes=['success', 'failure'])
	def open_drawera_cb(self):
		try:
			robot.omni_base.go_rel(-0.315, 0., 0.)

			#gripper.apply_force(0.9)
			robot.gripper.stop()
			robot.gripper.set_joint_value_target("hand_motor_joint", 1.)
			robot.gripper.go(wait=True)
			robot.gripper.stop()

			rospy.sleep(3)

			robot.omni_base.go_rel(-0.10, 0., 0.)

			#whole_body.move_to_go()
			robot.arm.stop()
			robot.arm.set_named_target('go')
			robot.arm.go(wait=True)
			robot.arm.stop()

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('OPENDRAWERA', smach.CBState(open_drawera_cb),
				transitions = {'success': 'MOVE2DRAWERB',
					       'failure': 'failure'})

	@smach.cb_interface(outcomes=['success', 'failure'],
			    input_keys=['fun_params', 'robot_info'],
			    output_keys=['fun_params'])
	def move_to_drawerb_cb(userdata):
		try:
			loc_key = "drawer top close"
			_pos = userdata.robot_info.location_list[loc_key.upper()].position
			rospy.loginfo("Opening {} at {}, {}, {}".format(loc_key, _pos.x, _pos.y, _pos.yaw))

			userdata.fun_params.robot_pose.delay = 1.0
			userdata.fun_params.robot_pose.tilt = -0.72
			userdata.fun_params.robot_pose.distance = _DISTANCE
			userdata.fun_params.robot_pose.height_diff = 0.25
			userdata.fun_params.robot_pose.wrist_roll = 0.0001 #1.57#0.0
			userdata.fun_params.robot_pose.in_use = True

			userdata.fun_params.edges_bb.position = _DISTANCE
			userdata.fun_params.edges_bb.orientation = 0.0001
			userdata.fun_params.edges_bb.maxx = 0.20
			userdata.fun_params.edges_bb.miny = 0.55
			userdata.fun_params.edges_bb.maxy = 0.65
			userdata.fun_params.edges_bb.maxz = 1.50
			userdata.fun_params.edges_bb.in_use = True

			userdata.fun_params.objects_bb.depth_min = 0.65
			userdata.fun_params.objects_bb.depth_max = 1.20
			userdata.fun_params.objects_bb.width = 0.30
			userdata.fun_params.objects_bb.height_min = _pos.z - 0.25
			userdata.fun_params.objects_bb.height_max = _pos.z + 0.025
			userdata.fun_params.objects_bb.in_use = True

			#omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)
			robot.omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)

			return 'success'

		except:
			return 'failure'

	smach.StateMachine.add('MOVE2DRAWERB', smach.CBState(move_to_drawerb_cb),
				transitions = {'success': 'ALIGN2DRAWERB',
					       'failure': 'OPENDRAWERERROR'})

	#Top drawer
	smach.StateMachine.add('ALIGN2DRAWERB', FindEdge(robot, align = True),
                               transitions = {'success': 'DETECTHANDLEB',
                                              'unknown': 'DETECTHANDLEB',
                                              'timeout': 'OPENDRAWERERROR',
                                              'failure': 'OPENDRAWERERROR'})
	
	smach.StateMachine.add('DETECTHANDLEB', DetectObjectOnVerticalPlane(robot, target = -0.04),
                               transitions = {'success': 'OPENDRAWERB',
                                              'unknown': 'OPENDRAWERERROR',
                                              'timeout': 'OPENDRAWERERROR',
                                              'failure': 'OPENDRAWERERROR'})

	#Performs all the hard-coded steps to grasp the object
	@smach.cb_interface(outcomes=['success', 'failure'])
	def open_drawerb_cb(self):
		try:
			robot.omni_base.go_rel(-0.20, 0., 0.)

			#gripper.apply_force(0.9)
			robot.gripper.stop()
			robot.gripper.set_joint_value_target("hand_motor_joint", 1.)
			robot.gripper.go(wait=True)
			robot.gripper.stop()

			rospy.sleep(3)

			robot.omni_base.go_rel(-0.10, 0., 0.)

			#whole_body.move_to_go()
			robot.arm.stop()
			robot.arm.set_named_target('go')
			robot.arm.go(wait=True)
			robot.arm.stop()

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('OPENDRAWERB', smach.CBState(open_drawerb_cb),
				transitions = {'success': 'MOVE2DRAWER2A',
					       'failure': 'OPENDRAWERERROR'})
	#Move to second drawer
	@smach.cb_interface(outcomes=['success', 'failure'],
			    input_keys=['fun_params', 'robot_info'],
			    output_keys=['fun_params'])
	def move_to_drawer2a_cb(userdata):
		try:
			loc_key = "drawer left close"
			_pos = userdata.robot_info.location_list[loc_key.upper()].position
			rospy.loginfo("Opening {} at {}, {}, {}".format(loc_key, _pos.x, _pos.y, _pos.yaw))

			userdata.fun_params.robot_pose.delay = 1.0
			userdata.fun_params.robot_pose.tilt = -0.96
			userdata.fun_params.robot_pose.distance = _DISTANCE
			userdata.fun_params.robot_pose.height_diff = 0.25
			userdata.fun_params.robot_pose.wrist_roll = 0.0001#0.0
			userdata.fun_params.robot_pose.in_use = True

			userdata.fun_params.edges_bb.position = _DISTANCE
			userdata.fun_params.edges_bb.orientation = 0.0001
			userdata.fun_params.edges_bb.maxx = 0.40
			userdata.fun_params.edges_bb.miny = 0.35
			userdata.fun_params.edges_bb.maxy = 0.45
			userdata.fun_params.edges_bb.maxz = 1.50
			userdata.fun_params.edges_bb.in_use = True

			userdata.fun_params.objects_bb.depth_min = 0.40
			userdata.fun_params.objects_bb.depth_max = 1.20
			userdata.fun_params.objects_bb.width = 0.30
			userdata.fun_params.objects_bb.height_min = _pos.z - 0.25
			userdata.fun_params.objects_bb.height_max = _pos.z + 0.025
			userdata.fun_params.objects_bb.in_use = True

			#omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)
			robot.omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)

			return 'success'

		except:
			return 'failure'

	smach.StateMachine.add('MOVE2DRAWER2A', smach.CBState(move_to_drawer2a_cb),
				transitions = {'success': 'ALIGN2DRAWER2A',
					       'failure': 'OPENDRAWERERROR'})

	smach.StateMachine.add('ALIGN2DRAWER2A', FindEdge(robot, align = True),
                               transitions = {'success': 'DETECTHANDLE2A',
                                              'unknown': 'DETECTHANDLE2A',
                                              'timeout': 'OPENDRAWERERROR',
                                              'failure': 'OPENDRAWERERROR'})
	
	smach.StateMachine.add('DETECTHANDLE2A', DetectObjectOnVerticalPlane(robot, target = -0.06),
                               transitions = {'success': 'OPENDRAWER2A',
                                              'unknown': 'OPENDRAWERERROR',
                                              'timeout': 'OPENDRAWERERROR',
                                              'failure': 'OPENDRAWERERROR'})

	#Performs all the hard-coded steps to grasp the object
	@smach.cb_interface(outcomes=['success', 'failure'])
	def open_drawer2a_cb(self):
		try:
			robot.omni_base.go_rel(-0.315, 0., 0.)

			#gripper.apply_force(0.9)
			robot.gripper.stop()
			robot.gripper.set_joint_value_target("hand_motor_joint", 1.)
			robot.gripper.go(wait=True)
			robot.gripper.stop()

			rospy.sleep(3)

			robot.omni_base.go_rel(-0.10, 0., 0.)

			#whole_body.move_to_go()
			robot.arm.stop()
			robot.arm.set_named_target('go')
			robot.arm.go(wait=True)
			robot.arm.stop()

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('OPENDRAWER2A', smach.CBState(open_drawer2a_cb),
				transitions = {'success': 'STARTTRIAL',
					       'failure': 'OPENDRAWERERROR'})

	@smach.cb_interface(outcomes=['success', 'failure'])
	def open_drawer_error_cb(userdata):
		try:
			if(_OPENDRAWER):
				rospy.loginfo('Sorry, I failed, please open the drawers for me.')

				#gripper.command(0.9)
				robot.gripper.stop()
				robot.gripper.set_joint_value_target("hand_motor_joint", 1.)
				robot.gripper.go(wait=True)
				robot.gripper.stop()

				rospy.sleep(3)

				#whole_body.move_to_go()
				robot.arm.stop()
				robot.arm.set_named_target('go')
				robot.arm.go(wait=True)
				robot.arm.stop()

				return 'success'
			else:
				return 'failure'
		except:
			return 'failure'

	smach.StateMachine.add('OPENDRAWERERROR', smach.CBState(open_drawer_error_cb),
				transitions = {'success': 'STARTTRIAL',
					       'failure': 'TIMETRANSITIONERROR'})
	##########
	#END: BASIC OPEN DRAWER FUNCTION
	##########

	##########
	#START: SEARCH LOCATION SENDER
	##########
	@smach.cb_interface(outcomes=['success', 'failure'],
			    input_keys=['search_pt'],
			    output_keys=['search_pt'])
	def start_trial_cb(userdata):
		try:
			userdata.search_pt += 1

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('STARTTRIAL', smach.CBState(start_trial_cb),
				transitions = {'success': 'MOVE2SEARCH',
					       'failure': 'TIMETRANSITIONERROR'})

	@smach.cb_interface(outcomes=['success', 'floor', 'furniture', 'next', 'failure'],
			    input_keys=['fun_params','search_pt','robot_info', 'location_name'],
			    output_keys=['fun_params'])
	def move_to_search_cb(userdata):
		try:
			if userdata.search_pt > len(userdata.robot_info.obsrv_pt_list):
				userdata.fun_params.robot_pose.in_use = False
				userdata.fun_params.edges_bb.in_use = False
				userdata.fun_params.objects_bb.in_use = False

				return 'success'

			current_pt = userdata.robot_info.obsrv_pt_list[userdata.search_pt - 1] 

			if userdata.location_name in current_pt.name:
				rospy.loginfo("Go to {}".format(current_pt.name))  

				robot.omni_base.go_abs(current_pt.position.x, current_pt.position.y, current_pt.position.yaw)

				userdata.fun_params.robot_pose.delay = 1.0
				userdata.fun_params.robot_pose.tilt = -0.70
				userdata.fun_params.robot_pose.distance = _DISTANCE
				userdata.fun_params.robot_pose.height_diff = 0.35
				userdata.fun_params.robot_pose.wrist_roll = 0.0
				userdata.fun_params.robot_pose.in_use = True

				userdata.fun_params.edges_bb.position = _DISTANCE
				userdata.fun_params.edges_bb.orientation = 0.0001
				userdata.fun_params.edges_bb.maxx = 0.40
				userdata.fun_params.edges_bb.miny = current_pt.position.z - 0.10
				userdata.fun_params.edges_bb.maxy = current_pt.position.z + 0.10
				userdata.fun_params.edges_bb.maxz = 1.50
				userdata.fun_params.edges_bb.in_use = True

				userdata.fun_params.objects_bb.depth_min = 0.40
				userdata.fun_params.objects_bb.depth_max = 1.20
				userdata.fun_params.objects_bb.width = 0.80
				userdata.fun_params.objects_bb.height_min = current_pt.position.z
				userdata.fun_params.objects_bb.height_max = current_pt.position.z + 0.40
				userdata.fun_params.objects_bb.in_use = True

				if "floor" in current_pt.name:
					userdata.fun_params.robot_pose.tilt = -0.96

					userdata.fun_params.objects_bb.depth_max = 0.90
					userdata.fun_params.objects_bb.width = 1.00
					userdata.fun_params.objects_bb.height_min = 0.0
					userdata.fun_params.objects_bb.height_max = 0.25

					return 'floor'

				if "tall" in current_pt.name:
					userdata.fun_params.robot_pose.tilt = -0.52

					userdata.fun_params.edges_bb.maxx = 0.50

					userdata.fun_params.objects_bb.depth_min = 0.40
					userdata.fun_params.objects_bb.depth_max = 1.20
					userdata.fun_params.objects_bb.width = 0.80

					return 'furniture'

				if "large" in current_pt.name:
					userdata.fun_params.robot_pose.tilt = -0.76

					userdata.fun_params.edges_bb.maxx = 0.50

					userdata.fun_params.objects_bb.depth_min = 0.40
					userdata.fun_params.objects_bb.depth_max = 1.20
					userdata.fun_params.objects_bb.width = 0.80

					return 'furniture'

				if "sofa" in current_pt.name:
					userdata.fun_params.robot_pose.tilt = -0.52

					userdata.fun_params.edges_bb.maxx = 0.50

					userdata.fun_params.objects_bb.depth_min = 0.40
					userdata.fun_params.objects_bb.depth_max = 1.25
					userdata.fun_params.objects_bb.width = 0.80

					return 'furniture'

				if "kitchen" in current_pt.name:
					userdata.fun_params.robot_pose.tilt = -0.52

					userdata.fun_params.edges_bb.maxx = 0.50

					userdata.fun_params.objects_bb.depth_min = 0.40
					userdata.fun_params.objects_bb.depth_max = 1.20
					userdata.fun_params.objects_bb.width = 0.80

					return 'furniture'
			else:
				return 'next'

		except:
			return 'failure'

	smach.StateMachine.add('MOVE2SEARCH', smach.CBState(move_to_search_cb),
				transitions = {'success': 'success',
					       'floor': 'DETECTGRASPOBJECTONFLOOR',
					       'furniture':'ALIGN2FURNITURE',
					       'next':'STARTTRIAL',
					       'failure': 'TIMETRANSITIONERROR'})
	##########
	#END: SEARCH LOCATION SENDER
	##########

	##########
	#START: BASIC FLOOR GRASPING FUNCTION
	##########
	smach.StateMachine.add('DETECTGRASPOBJECTONFLOOR', DetectGraspObjectOnFloor(robot),
                               transitions = {'success': 'MOVE2DEPOSIT',
                                              'unknown': 'STARTTRIAL',
                                              'timeout': 'TIMETRANSITIONERROR',
                                              'failure': 'TIMETRANSITIONERROR'})
	##########
	#END: BASIC FLOOR GRASPING FUNCTION
	##########

	##########
	#START: BASIC FURNITURE GRASPING FUNCTION
	##########
	smach.StateMachine.add('ALIGN2FURNITURE', FindEdge(robot, align = _ALIGN),
                               transitions = {'success': 'DETECTOBJECTSONFURNITURE',
                                              'unknown': 'DETECTOBJECTSONFURNITURE',
                                              'timeout': 'DETECTOBJECTSONFURNITURE',
                                              'failure': 'TIMETRANSITIONERROR'})

	smach.StateMachine.add('DETECTOBJECTSONFURNITURE', DetectObjectOnPlane(robot),
                               transitions = {'success': 'TAKEOBJECTONFURNITURE',
                                              'unknown': 'STARTTRIAL',
                                              'timeout': 'TIMETRANSITIONERROR',
                                              'failure': 'TIMETRANSITIONERROR'})

	#Performs all the hard-coded steps to grasp the object
	@smach.cb_interface(outcomes=['success', 'failure'])
	def take_object_furniture_cb(self):
		try:
			#whole_body.move_end_effector_pose(((0.02, 0, 0.0), (0, 0, 0, 1)), 'hand_palm_link')
			#whole_body.move_end_effector_pose(((0, 0, -0.25), (0, 0, 0, 1)), 'hand_palm_link')
			robot.omni_base.go_rel(-0.25, 0., 0.)

			#whole_body.move_to_go()
			robot.arm.stop()
			robot.arm.set_named_target('go')
			robot.arm.go(wait=True)
			robot.arm.stop()

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('TAKEOBJECTONFURNITURE', smach.CBState(take_object_furniture_cb),
				transitions = {'success': 'MOVE2DEPOSIT',
					       'failure': 'TIMETRANSITIONERROR'})
	##########
	#END: BASIC FURNITURE GRASPING FUNCTION
	##########

	##########
	#START: DEPOSIT LOCATION SENDER
	##########
	@smach.cb_interface(outcomes=['success', 'drawertop', 'drawerbottom', 'container', 'tray', 'basket', 'failure'],
			    input_keys=['fun_params','search_pt','robot_info', 'obj_name', 'location_name'],
			    output_keys=['fun_params'])
	def move_to_deposit_cb(userdata):
		try:
			userdata.fun_params.robot_pose.delay = 1.0
			userdata.fun_params.robot_pose.tilt = -0.70
			userdata.fun_params.robot_pose.distance = _DISTANCE
			userdata.fun_params.robot_pose.height_diff = 0.35
			userdata.fun_params.robot_pose.wrist_roll = 0.0
			userdata.fun_params.robot_pose.in_use = True

			userdata.fun_params.edges_bb.position = _DISTANCE
			userdata.fun_params.edges_bb.orientation = 0.0001
			userdata.fun_params.edges_bb.maxx = 0.40
			userdata.fun_params.edges_bb.miny = 0.40
			userdata.fun_params.edges_bb.maxy = 0.60
			userdata.fun_params.edges_bb.maxz = 1.50
			userdata.fun_params.edges_bb.in_use = True

			userdata.fun_params.points_bb.maxx = 0.40
			userdata.fun_params.points_bb.miny = 0.40
			userdata.fun_params.points_bb.maxy = 0.60
			userdata.fun_params.points_bb.maxz = 1.50
			userdata.fun_params.points_bb.in_use = True

			userdata.fun_params.objects_bb.depth_min = 0.40
			userdata.fun_params.objects_bb.depth_max = 1.20
			userdata.fun_params.objects_bb.width = 0.80
			userdata.fun_params.objects_bb.height_min = 0.40
			userdata.fun_params.objects_bb.height_max = 0.80
			userdata.fun_params.objects_bb.in_use = True

			userdata.fun_params.space_info.dist2plane = 0.08
			userdata.fun_params.space_info.wside = 0.10
			userdata.fun_params.space_info.dside = 0.12
			userdata.fun_params.space_info.delta_lift = 0.10
			userdata.fun_params.space_info.side = 0
			userdata.fun_params.space_info.in_use = True

			obj_key = userdata.obj_name.upper()
			#obj_key = "unknown"
			location = userdata.robot_info.obj_list[obj_key.upper()].defaultLocation

			#location = "tray 1"
			if "drawer left open" in location:
				_pos = userdata.robot_info.location_list[location.upper()].position

				userdata.fun_params.robot_pose.tilt = -0.78

				userdata.fun_params.edges_bb.maxx = 0.40
				userdata.fun_params.edges_bb.miny = _pos.z - 0.20
				userdata.fun_params.edges_bb.maxy = _pos.z

				userdata.fun_params.points_bb.maxx = 0.40
				userdata.fun_params.points_bb.miny = _pos.z - 0.20
				userdata.fun_params.points_bb.maxy = _pos.z

				rospy.loginfo("Move to {} at {}, {}, {}".format(location, _pos.x, _pos.y, _pos.yaw))
				#omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)
				robot.omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)

				return 'drawerbottom'

			elif "drawer bottom open" in location:
				_pos = userdata.robot_info.location_list[location.upper()].position

				userdata.fun_params.robot_pose.tilt = -0.78

				userdata.fun_params.edges_bb.maxx = 0.40
				userdata.fun_params.edges_bb.miny = _pos.z - 0.20
				userdata.fun_params.edges_bb.maxy = _pos.z

				userdata.fun_params.points_bb.maxx = 0.40
				userdata.fun_params.points_bb.miny = _pos.z - 0.20
				userdata.fun_params.points_bb.maxy = _pos.z

				rospy.loginfo("Move to {} at {}, {}, {}".format(location, _pos.x, _pos.y, _pos.yaw))
				#omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)
				robot.omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)

				return 'drawerbottom'

			elif "drawer top open" in location:
				_pos = userdata.robot_info.location_list[location.upper()].position

				userdata.fun_params.edges_bb.maxx = 0.40
				userdata.fun_params.edges_bb.miny = _pos.z - 0.20
				userdata.fun_params.edges_bb.maxy = _pos.z

				userdata.fun_params.points_bb.maxx = 0.40
				userdata.fun_params.points_bb.miny = _pos.z - 0.20
				userdata.fun_params.points_bb.maxy = _pos.z

				rospy.loginfo("Move to {} at {}, {}, {}".format(location, _pos.x, _pos.y, _pos.yaw))
				#omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)
				robot.omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)

				return 'drawertop'

			elif "container" in location:
				_pos = userdata.robot_info.location_list[location.upper()].position

				userdata.fun_params.robot_pose.tilt = -0.78
				userdata.fun_params.edges_bb.orientation = []

				userdata.fun_params.edges_bb.maxx = 0.15
				userdata.fun_params.edges_bb.miny = _pos.z - 0.05
				userdata.fun_params.edges_bb.maxy = _pos.z

				userdata.fun_params.points_bb.maxx = 0.15
				userdata.fun_params.points_bb.miny = _pos.z - 0.05
				userdata.fun_params.points_bb.maxy = _pos.z

				rospy.loginfo("Move to {} at {}, {}, {}".format(location, _pos.x, _pos.y, _pos.yaw))
				#omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)
				robot.omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)

				return 'container'

			elif "tray" in location:
				_pos = userdata.robot_info.location_list[location.upper()].position

				userdata.fun_params.robot_pose.tilt = -0.78

				userdata.fun_params.edges_bb.maxx = 0.15
				userdata.fun_params.edges_bb.miny = _pos.z - 0.10
				userdata.fun_params.edges_bb.maxy = _pos.z + 10

				userdata.fun_params.objects_bb.width = 0.40
				userdata.fun_params.objects_bb.height_max = 0.60

				rospy.loginfo("Move to {} at {}, {}, {}".format(location, _pos.x, _pos.y, _pos.yaw))
				#omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)
				robot.omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)

				return 'tray'

			elif "basket" in location:
				_pos = userdata.robot_info.location_list[location.upper()].position

				userdata.fun_params.robot_pose.tilt = -0.78
				userdata.fun_params.edges_bb.orientation = []

				userdata.fun_params.edges_bb.maxx = 0.15
				userdata.fun_params.edges_bb.miny = _pos.z - 0.10
				userdata.fun_params.edges_bb.maxy = _pos.z + 0.05

				userdata.fun_params.points_bb.maxx = 0.15
				userdata.fun_params.points_bb.miny = _pos.z - 0.10
				userdata.fun_params.points_bb.maxy = _pos.z + 0.05

				rospy.loginfo("Move to {} at {}, {}, {}".format(location, _pos.x, _pos.y, _pos.yaw))
				#omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)
				robot.omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)

				return 'basket'

			elif "unknown" in location:
				location = location + " " + userdata.location_name
				_pos = userdata.robot_info.location_list[location.upper()].position

				userdata.fun_params.robot_pose.tilt = -0.78
				userdata.fun_params.edges_bb.orientation = []

				userdata.fun_params.edges_bb.maxx = 0.15
				userdata.fun_params.edges_bb.miny = _pos.z - 0.05
				userdata.fun_params.edges_bb.maxy = _pos.z + 0.05

				userdata.fun_params.points_bb.maxx = 0.15
				userdata.fun_params.points_bb.miny = _pos.z - 0.05
				userdata.fun_params.points_bb.maxy = _pos.z + 0.05

				rospy.loginfo("Move to {} at {}, {}, {}".format(location, _pos.x, _pos.y, _pos.yaw))
				#omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)
				robot.omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)

				return 'basket'
		except:
			return 'failure'

	smach.StateMachine.add('MOVE2DEPOSIT', smach.CBState(move_to_deposit_cb),
				transitions = {'success': 'TRANSITIONOK',
					       'drawertop': 'FINDBASKET',#'ALIGNTOP',
					       'drawerbottom': 'FINDBASKET',#'ALIGNBOTTOM',
					       'container': 'FINDBASKET',#'FINDCONTAINER',
					       'tray': 'FINDBASKET',#'FINDTRAY',
					       'basket': 'FINDBASKET',#'FINDBASKET',
					       'failure': 'TIMETRANSITIONERROR'})
	##########
	#END: DEPOSIT LOCATION SENDER
	##########


	##########
	#START: BASIC DELIVERY2BASKET FUNCTION
	##########
	smach.StateMachine.add('FINDBASKET', FindEdge(robot, align = _ALIGN),
                               transitions = {'success': 'DEPOSIT2BASKET',
                                              'unknown': 'DEPOSIT2BASKET',
                                              'timeout': 'DEPOSIT2BASKET',
                                              'failure': 'TIMETRANSITIONERROR'})

	#Performs all the hard-coded steps to grasp the object
	@smach.cb_interface(outcomes=['success', 'failure'])
	def take_object_furniture_cb(self):
		try:
			robot.omni_base.go_rel(0.60, 0., 0.)
			#arm_flex_joint+wrist_flex_joint=3.14
			robot.arm.stop()
			target_joints = {'arm_flex_joint': -1.50,
					 'arm_lift_joint': 0.15,
					 'arm_roll_joint': 0.0,
					 'head_pan_joint': 0.0,
					 'head_tilt_joint': -0.76,
					 'wrist_flex_joint': -1.62,
					 'wrist_roll_joint': 0.0}
			robot.arm.go(target_joints,wait=True)
			robot.arm.stop()
			robot.arm.clear_pose_targets()

			#gripper.command(0.9)
			robot.gripper.stop()
			robot.gripper.set_joint_value_target("hand_motor_joint", 1.)
			robot.gripper.go(wait=True)
			robot.gripper.stop()

			rospy.sleep(4)

			#whole_body.move_to_go()
			robot.arm.stop()
			robot.arm.set_named_target('go')
			robot.arm.go(wait=True)
			robot.arm.stop()

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('DEPOSIT2BASKET', smach.CBState(take_object_furniture_cb),
				transitions = {'success': 'MOVE2SEARCH',
					       'failure': 'TIMETRANSITIONERROR'})
	##########
	#END: BASIC DELIVERY2BASKET FUNCTION
	##########

	##########
	#START: BASIC TAKE DRINK FUNCTION
	##########
	@smach.cb_interface(outcomes=['success', 'failure'])
	def time_transition_error_cb(userdata):
		try:
			#gripper.command(0.9)
			robot.gripper.stop()
			robot.gripper.set_joint_value_target("hand_motor_joint", 1.)
			robot.gripper.go(wait=True)
			robot.gripper.stop()

			#whole_body.move_to_go()
			robot.arm.stop()
			robot.arm.set_named_target('go')
			robot.arm.go(wait=True)
			robot.arm.stop()

			rospy.loginfo('Sorry, something happened.')

			rospy.sleep(4)

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('TIMETRANSITIONERROR', smach.CBState(time_transition_error_cb),
				transitions = {'success': 'TRANSITIONOK',
					       'failure': 'failure'})

	@smach.cb_interface(outcomes=['success', 'wait', 'failure'],
			    input_keys=['start_time', 'stop_time','timetransition'],
			    output_keys=['timetransition'])
	def time_transition_cb(userdata):
		try:
			#gripper.command(0.9)
			robot.gripper.stop()
			robot.gripper.set_joint_value_target("hand_motor_joint", 1.)
			robot.gripper.go(wait=True)
			robot.gripper.stop()

			#whole_body.move_to_go()
			robot.arm.stop()
			robot.arm.set_named_target('go')
			robot.arm.go(wait=True)
			robot.arm.stop()

			if (userdata.timetransition == False):
				rospy.loginfo('The time is over.')
				rospy.loginfo('I will wait for the right time to start the next task.')
				rospy.sleep(2)

				userdata.timetransition = True

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval >= _TIMETASK1 - 15:
					return 'success'
				else:
					return 'wait'
		except:
			return 'failure'

	smach.StateMachine.add('TIMETRANSITION', smach.CBState(time_transition_cb),
				transitions = {'success': 'MOVE2LIVING1',
					       'wait': 'TIMETRANSITION',
					       'failure': 'failure'})

	@smach.cb_interface(outcomes=['success', 'wait', 'failure'],
			    input_keys=['start_time', 'stop_time','timetransition'],
			    output_keys=['timetransition'])
	def transition_ok_cb(userdata):
		try:
			#gripper.command(0.9)
			robot.gripper.stop()
			robot.gripper.set_joint_value_target("hand_motor_joint", 1.)
			robot.gripper.go(wait=True)
			robot.gripper.stop()

			#whole_body.move_to_go()
			robot.arm.stop()
			robot.arm.set_named_target('go')
			robot.arm.go(wait=True)
			robot.arm.stop()

			if (userdata.timetransition == False):
				rospy.loginfo('I will wait for the right time to start the next task.')
				rospy.sleep(2)

				userdata.timetransition = True

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval >= _TIMETASK1 - 15:
					return 'success'
				else:
					return 'wait'
		except:
			return 'failure'

	smach.StateMachine.add('TRANSITIONOK', smach.CBState(transition_ok_cb),
				transitions = {'success': 'MOVE2LIVING1',
					       'wait': 'TRANSITIONOK',
					       'failure': 'failure'})

	@smach.cb_interface(outcomes=['success', 'failure'],
			    input_keys=['start_time', 'fun_params','robot_info'],
			    output_keys=['start_time', 'fun_params'])
	def move_to_living1_cb(userdata):
		try:
			loc_key = "living room entrance"
			_pos = userdata.robot_info.location_list[loc_key.upper()].position
			rospy.loginfo("Move to {} at {}, {}, {}".format(loc_key, _pos.x, _pos.y, _pos.yaw))
			robot.omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)

			rospy.loginfo('I will bring you your order.')

			userdata.fun_params.robot_pose.in_use = False
			userdata.fun_params.edges_bb.in_use = False
			userdata.fun_params.objects_bb.in_use = False
			userdata.fun_params.space_info.in_use = False

			userdata.start_time = time.time()

			return 'success'

		except:
			return 'failure'

	smach.StateMachine.add('MOVE2LIVING1', smach.CBState(move_to_living1_cb),
				transitions = {'success': 'STARTTRIAL2',
					       'failure': 'failure'})

	@smach.cb_interface(outcomes=['success', 'failure'],
			    input_keys=['counter', 'trials'],
			    output_keys=['counter', 'trials'])
	def start_trial2_cb(userdata):
		try:
			userdata.counter = 0
			userdata.trials = 0

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('STARTTRIAL2', smach.CBState(start_trial2_cb),
				transitions = {'success': 'MOVE2SHELF',
					       'failure': 'failure'})

	@smach.cb_interface(outcomes=['success', 'failure'],
			    input_keys=['fun_params','robot_info'],
			    output_keys=['fun_params'])
	def move_to_shelf_cb(userdata):
		try:
			loc_key = "shelf middle"
			_pos = userdata.robot_info.location_list[loc_key.upper()].position
			rospy.loginfo("Move to {} at {}, {}, {}".format(loc_key, _pos.x, _pos.y, _pos.yaw))
			robot.omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)

			#Bypass navigation
			#q = quaternion_from_euler(0, 0, _pos.yaw, 'rxyz')
			#q = Quaternion(q[0], q[1], q[2], q[3])

			#goal = MoveBaseGoal()
			#goal.target_pose.header.frame_id = "map"
			#goal.target_pose.pose.position.x = _pos.x
			#goal.target_pose.pose.position.y = _pos.y
			#goal.target_pose.pose.orientation = quaternion_from_euler(0, 0, q)

			#robot.navclient.send_goal(goal)
			#robot.navclient.wait_for_result()
			###

			userdata.fun_params.robot_pose.delay = 1.0
			userdata.fun_params.robot_pose.tilt = -0.34
			userdata.fun_params.robot_pose.distance = _DISTANCE
			userdata.fun_params.robot_pose.height_diff = 0.45
			userdata.fun_params.robot_pose.wrist_roll = 0.0001#-1.57
			userdata.fun_params.robot_pose.in_use = True

			userdata.fun_params.edges_bb.position = _DISTANCE
			userdata.fun_params.edges_bb.orientation = 0.0001
			userdata.fun_params.edges_bb.maxx = 0.50
			userdata.fun_params.edges_bb.miny = _pos.z - 0.10
			userdata.fun_params.edges_bb.maxy = _pos.z + 0.10
			userdata.fun_params.edges_bb.maxz = 1.50
			userdata.fun_params.edges_bb.in_use = True

			userdata.fun_params.objects_bb.depth_min = 0.40
			userdata.fun_params.objects_bb.depth_max = 1.20
			userdata.fun_params.objects_bb.width = 1.00
			userdata.fun_params.objects_bb.height_min = _pos.z
			userdata.fun_params.objects_bb.height_max = _pos.z + 0.20
			userdata.fun_params.objects_bb.in_use = True

			return 'success'

		except:
			return 'failure'

	smach.StateMachine.add('MOVE2SHELF', smach.CBState(move_to_shelf_cb),
				transitions = {'success': 'ALIGN2SHELF',
					       'failure': 'failure'})

	smach.StateMachine.add('ALIGN2SHELF', FindEdge(robot, align = _ALIGN),
                               transitions = {'success': 'DETECTOBJECTSONSHELF',
                                              'unknown': 'DETECTOBJECTSONSHELF',
                                              'timeout': 'failure',
                                              'failure': 'failure'})

	##TODO: Add comments
	smach.StateMachine.add('DETECTOBJECTSONSHELF', DetectObjectOnPlane(robot, target = 0.00),
                               transitions = {'success': 'TAKEOBJECTONSHELF',
                                              'unknown': 'failure',
                                              'timeout': 'failure',
                                              'failure': 'failure'})

	#Performs all the hard-coded steps to grasp the object
	@smach.cb_interface(outcomes=['success', 'failure'])
	def take_object_shelf_cb(self):
		try:
			#whole_body.move_end_effector_pose(((0.02, 0, 0.0), (0, 0, 0, 1)), 'hand_palm_link')
			#whole_body.move_end_effector_pose(((0, 0, -0.25), (0, 0, 0, 1)), 'hand_palm_link')
			robot.omni_base.go_rel(-0.25, 0., 0.)

			#whole_body.move_to_go()
			robot.arm.stop()
			robot.arm.set_named_target('go')
			robot.arm.go(wait=True)
			robot.arm.stop()

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('TAKEOBJECTONSHELF', smach.CBState(take_object_shelf_cb),
				transitions = {'success': 'MOVE2LIVING2',
					       'failure': 'failure'})
	##########
	#END: BASIC TAKE DRINK FUNCTION
	##########

	##########
	#START: BASIC DELIVERY DRINK FUNCTION
	##########
	@smach.cb_interface(outcomes=['success', 'failure'],
			    input_keys=['fun_params','robot_info'],
			    output_keys=['fun_params'])
	def move_to_living2_cb(userdata):
		try:
			loc_key = "living room delivery"
			_pos = userdata.robot_info.location_list[loc_key.upper()].position
			rospy.loginfo("Move to {} at {}, {}, {}".format(loc_key, _pos.x, _pos.y, _pos.yaw))
			robot.omni_base.go_abs(_pos.x, _pos.y, _pos.yaw)

			userdata.fun_params.robot_pose.in_use = False
			userdata.fun_params.edges_bb.in_use = False
			userdata.fun_params.objects_bb.in_use = False
			userdata.fun_params.space_info.in_use = False

			#whole_body.move_to_neutral()
			robot.arm.stop()
			robot.arm.set_named_target('neutral')
			robot.arm.go(wait=True)
			robot.arm.stop()

			rospy.loginfo('Here is your request. Now I will find you.')

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('MOVE2LIVING2', smach.CBState(move_to_living2_cb),
				transitions = {'success': 'DELIVERY',
					       'failure': 'failure'})

	@smach.cb_interface(outcomes=['success', 'failure'])
	def delivery_cb(self):
		try:
			robot.omni_base.go_rel(0.65, 0., 0.)

			if random.randint(0,2) == 0:
				robot.omni_base.go_rel(0., 0., 0.76)
			else:
				robot.omni_base.go_rel(0., 0., -0.76)

			robot.arm.stop()
			target_joints = {'arm_flex_joint': -1.0,
					 'arm_lift_joint': 0.25,
					 'arm_roll_joint': 0.0,
					 'head_pan_joint': 0.0,
					 'head_tilt_joint': -0.76,
					 'wrist_flex_joint': -0.52,
					 'wrist_roll_joint': 0.0}
			robot.arm.go(target_joints,wait=True)
			robot.arm.stop()
			robot.arm.clear_pose_targets()

			#gripper.command(0.9)
			robot.gripper.stop()
			robot.gripper.set_joint_value_target("hand_motor_joint", 1.)
			robot.gripper.go(wait=True)
			robot.gripper.stop()

			rospy.sleep(4)

			#whole_body.move_to_go()
			robot.arm.stop()
			robot.arm.set_named_target('go')
			robot.arm.go(wait=True)
			robot.arm.stop()

			robot.omni_base.go_rel(0.50, 0., 0.)

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('DELIVERY', smach.CBState(delivery_cb),
				transitions = {'success': 'success',
					       'failure': 'failure'})
	##########
	#START: BASIC DELIVERY DRINK FUNCTION
	##########

	##########

  return sm

sm = create_sm()

outcome = sm.execute()

if outcome == 'success':
	#SAY('I finished the task.')
	#rospy.sleep(2)
	rospy.loginfo('I finished the task.')
else:
	#SAY('Sorry, something happend. I did not finish the task.')
	rospy.signal_shutdown('Some error occured.')
