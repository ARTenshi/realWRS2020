#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import smach
import actionlib
import tf2_ros
import math
import numpy as np
import traceback
import time

import hsrb_interface
from hsrb_interface import geometry
from modules import FunctionParams

class GraspVisual(smach.State):
	def __init__(self, robot, distance = 0.80, target = 0.10, neutral = True, tf_buffer=None, timeout=None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
					   input_keys = ['start_time', 'stop_time'])

		self.robot = robot

		#if tf_buffer:
		#	self.tf_buffer = tf_buffer
		#else:
		#	self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
		#	tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.gripper = self.robot.get('gripper')

		self.distance = distance
		self.target = target
		self.neutral = neutral

	def execute(self, userdata):
		try:
			#Gripper oppening. Roughly 1.2[rad] is the open position, 0.0 [rad] is the closed position.
			self.gripper.command(0.9)

			#rospy.sleep(5)

			arm_flex_angle =  self.whole_body.joint_positions['arm_flex_joint']
			rospy.loginfo('arm_distance: {}'.format( 0.345*math.sin(abs(arm_flex_angle)) ) )

			#End-effector to target distance. From the Find Object/Space state, the robot origin is at "distance" to the target
			#and therefore, the end effector is at "distance - 0.28" in the neutral position
			#and at "distance - 0.13" in the go position
			if (self.neutral):
				gripper2target_distance = self.distance - 0.28
			else:
				gripper2target_distance = self.distance - 0.13
 
			#The self.target distance is the distance from the origin of the end effector to the target in the direction of the target
			#0.08 for the kitchen's door (gripper closesd, only the tip of the gripper), 
			#0.04 for the short handle and the fridge's handle, 0.02 for the long handle
			#gripper2target distance is the distance from the end effector current position to the target position
			gripper2target_distance = gripper2target_distance - self.target

			#Robot-to-target distance. Distance from the end-effector to the target after height alignment,
			#wher the gripper moved forward
			diff_dist = gripper2target_distance - 0.345*math.sin( abs(arm_flex_angle) )

			linear_weight = self.whole_body.linear_weight
			angular_weight = self.whole_body.angular_weight

			self.whole_body.linear_weight = 1
			self.whole_body.angular_weight = 100

			self.whole_body.move_end_effector_pose(((0, 0, diff_dist), (0, 0, 0, 1)), 'hand_palm_link')

			self.whole_body.linear_weight = linear_weight
			self.whole_body.angular_weight = angular_weight

			self.gripper.apply_force(1.0)

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

class GraspDeltaVisual(smach.State):
	def __init__(self, robot, distance = 0.80, tf_buffer=None, timeout=None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
					   input_keys = ['fun_params', 'objects', 'obj_plane', 'start_time', 'stop_time'])

		self.robot = robot

		#if tf_buffer:
		#	self.tf_buffer = tf_buffer
		#else:
		#	self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
		#	tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.gripper = self.robot.get('gripper')

		self.distance = distance

	def execute(self, userdata):
		try:
			frontal = True
			if (userdata.obj_plane < 0.05):
				frontal = False


			#x positive to the front
			#y positive to the left
			#z positive upwards
			if (frontal):
				delta_x = 0.0
				delta_y = -0.02
				delta_z = 0.0
			else:
				delta_x = -0.19
				delta_y = -0.08
				delta_z = 0.19


			#Gripper oppening. Roughly 1.2[rad] is the open position, 0.0 [rad] is the closed position.
			self.gripper.command(0.9)

			arm_lift_height =  self.whole_body.joint_positions['arm_lift_joint']
			arm_flex_angle =  self.whole_body.joint_positions['arm_flex_joint']

			#rospy.sleep(5)
			if (not frontal):
				self.whole_body.move_to_joint_positions({'arm_lift_joint': arm_lift_height + 0.25})

			rospy.loginfo('arm_distance: {}'.format( 0.345*math.sin(abs(arm_flex_angle)) ) )

			#End-effector to target distance. From the Find Object/Space state, the robot origin is at "distance" to the target
			#and therefore, the end effector is at "distance - 0.28" in the neutral position
			gripper2target_distance = self.distance - 0.28
 
			#The self.target distance is the distance from the origin of the end effector to the target in the direction of the target
			#0.08 for the kitchen's door (gripper closesd, only the tip of the gripper), 
			#0.04 for the short handle and the fridge's handle, 0.02 for the long handle
			#gripper2target distance is the distance from the end effector current position to the target position
			gripper2target_distance = gripper2target_distance - delta_x

			#Robot-to-target distance. Distance from the end-effector to the target after height alignment,
			#wher the gripper moved forward
			diff_dist = gripper2target_distance - 0.345*math.sin( abs(arm_flex_angle) )

			linear_weight = self.whole_body.linear_weight
			angular_weight = self.whole_body.angular_weight

			self.whole_body.linear_weight = 1
			self.whole_body.angular_weight = 100

			self.whole_body.move_end_effector_pose(((0, delta_y, diff_dist), (0, 0, 0, 1)), 'hand_palm_link')

			self.whole_body.linear_weight = linear_weight
			self.whole_body.angular_weight = angular_weight

			rospy.loginfo('wrist_angle: {}, wrist_flex: {}'.format( -1*(np.pi+arm_flex_angle), arm_flex_angle ) )
			if (not frontal):
				rot_angle = np.pi+arm_flex_angle
				if (rot_angle > 1.92):
					rot_angle = 1.92
				self.whole_body.move_to_joint_positions({'wrist_flex_joint': -1*rot_angle})
				self.whole_body.move_to_joint_positions({'arm_lift_joint': arm_lift_height + delta_z})

			self.gripper.apply_force(1.0)

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

class GraspTactile(smach.State):
	def __init__(self, robot, target = 0.10, neutral = True, tf_buffer=None, timeout=None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
					   input_keys = ['start_time', 'stop_time'])

		self.robot = robot

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		self.whole_body = self.robot.get('whole_body')
		self.gripper = self.robot.get('gripper')

		self.target = target
		self.neutral = neutral

	def execute(self, userdata):
		try:
			#Gripper oppening. Roughly 1.2[rad] is the open position, 0.0 [rad] is the closed position.
			gripper.command(0.9)

			pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
			twist = Twist()
			twist.linear.x = 0.05
			while not rospy.is_shutdown():
				try:
					wrench = rospy.wait_for_message('/hsrb/wrist_wrench/raw', WrenchStamped, timeout=1.).wrench
					#print('force: {}'.format(wrench.force.z))
					if wrench.force.z > .5:
						break
					pub.publish(twist)
				except:
					traceback.print_exc()
					print('force is not comming')
			pub.publish(Twist())

			#The self.target distance is the distance from the origin of the end effector to the target in the direction of the target
			#0.08 for the kitchen's door (gripper closesd, only the tip of the gripper), 
			#0.04 for the short handle and the fridge's handle, 0.02 for the long handle
			#gripper2target distance is the distance from the end effector current position to the target position
			#The drift is added due to the additional distance that the robot moved forward after pushing
			target_dist = -1*(self.target + 0.02)

			linear_weight = self.whole_body.linear_weight
			angular_weight = self.whole_body.angular_weight

			self.whole_body.linear_weight = 1
			self.whole_body.angular_weight = 100

			self.whole_body.move_end_effector_pose(((0, 0, target_dist), (0, 0, 0, 1)), 'hand_palm_link')

			self.whole_body.linear_weight = linear_weight
			self.whole_body.angular_weight = angular_weight

			self.gripper.apply_force(1.0)

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

class DeliverVisual(smach.State):
	def __init__(self, robot, distance = 0.80, target = 0.00, neutral = True, delta_flex = 0.0, tf_buffer=None, timeout=None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
					   input_keys = ['start_time', 'stop_time'])

		self.robot = robot

		#if tf_buffer:
		#	self.tf_buffer = tf_buffer
		#else:
		#	self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
		#	tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.gripper = self.robot.get('gripper')

		self.distance = distance
		self.target = target
		self.neutral = neutral
		self.delta_flex = delta_flex

	def execute(self, userdata):
		try:
			arm_flex_angle =  self.whole_body.joint_positions['arm_flex_joint']
			rospy.loginfo('arm_distance: {}'.format( 0.345*math.sin(abs(arm_flex_angle)) ) )

			#End-effector to target distance. From the Find Object/Space state, the robot origin is at "distance" to the target
			#and therefore, the end effector is at "distance - 0.28" in the neutral position
			#and at "distance - 0.13" in the go position
			if (self.neutral):
				gripper2target_distance = self.distance - 0.28
			else:
				gripper2target_distance = self.distance - 0.13
 
			#The self.target distance is the distance from the origin of the end effector to the target in the direction of the target
			#0.08 for the kitchen's door (gripper closesd, only the tip of the gripper), 
			#0.04 for the short handle and the fridge's handle, 0.02 for the long handle
			#gripper2target distance is the distance from the end effector current position to the target position
			gripper2target_distance = gripper2target_distance - self.target

			#Robot-to-target distance. Distance from the end-effector to the target after height alignment,
			#where the gripper moved forward
			diff_dist = gripper2target_distance - 0.345*math.sin( abs(arm_flex_angle) )

			linear_weight = self.whole_body.linear_weight
			angular_weight = self.whole_body.angular_weight

			self.whole_body.linear_weight = 1
			self.whole_body.angular_weight = 100

			self.whole_body.move_end_effector_pose(((0, 0, diff_dist), (0, 0, 0, 1)), 'hand_palm_link')

			#Gripper oppening. Roughly 1.2[rad] is the open position, 0.0 [rad] is the closed position.
			self.gripper.command(0.9)

			wrist_flex_joint =  self.whole_body.joint_positions['wrist_flex_joint']
			self.whole_body.move_to_joint_positions({'wrist_flex_joint': wrist_flex_joint + self.delta_flex})
			self.whole_body.move_to_joint_positions({'wrist_flex_joint': wrist_flex_joint})

			#self.whole_body.move_end_effector_pose(((0, 0, -1*(diff_dist + 0.15)), (0, 0, 0, 1)), 'hand_palm_link')
			self.whole_body.move_end_effector_pose(((0, 0, -1*(diff_dist)), (0, 0, 0, 1)), 'hand_palm_link')

			self.whole_body.linear_weight = linear_weight
			self.whole_body.angular_weight = angular_weight

			self.whole_body.move_to_neutral()

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

