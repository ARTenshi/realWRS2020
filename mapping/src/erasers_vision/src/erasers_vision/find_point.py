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

#from geometry_msgs.msg import Twist

#from common import HSRB_Xtion

import hsrb_interface
from hsrb_interface import geometry

#LineFinder libraries
from erasers_nav_msgs.srv import GetClosestPoint, GetLinePoint
from modules import FunctionParams

#Attempts to align the manipulator close to the a salient point using visual feedback
#tilt: head tilt angle
#delay: to update camera buffer
#wrist_roll: hand rotation
#(miny,maxy): handle valid height region (negative values uses default yaml file configuration)
class FindPoint(smach.State):
	def __init__(self, robot, delay = -.0, tilt = -0.70, distance = 0.80, height_diff = 0.30, wrist_roll = 0.0, maxx = -1.0, miny = -1.0, maxy = -1.0, maxz = -1.0, drift_z = 0.00, tf_buffer = None, timeout = None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown', 'timeout'],
					   input_keys = ['fun_params', 'start_time', 'stop_time'])

		self.robot = robot
		#Connect to base
		#self.move_base_pub = rospy.Publisher('/hsrb/command_velocity', Twist)

		#Start LineFinder client
		rospy.wait_for_service('/erasers/navigation/front_point_srv')
		self.get_closest_point = rospy.ServiceProxy('/erasers/navigation/front_point_srv', GetClosestPoint)

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.omni_base = self.robot.get("omni_base")

		self.delay = delay
		self.tilt = tilt
		self.distance = distance
		self.height_diff = height_diff
		self.wrist_roll = wrist_roll

		self.maxx = maxx
		self.miny = miny
		self.maxy = maxy
		self.maxz = maxz

		self.drift_z = drift_z

	def execute(self, userdata):
		try:
			robot_pose = userdata.fun_params.robot_pose
			if (robot_pose.in_use):
				if (robot_pose.delay):
					self.delay = robot_pose.delay

				if (robot_pose.tilt):
					self.tilt = robot_pose.tilt

				if (robot_pose.distance):
					self.distance = robot_pose.distance

				if (robot_pose.wrist_roll):
					self.height_diff = robot_pose.height_diff

				if (robot_pose.wrist_roll):
					self.wrist_roll = robot_pose.wrist_roll

			points_bb = userdata.fun_params.points_bb
			if (points_bb.in_use):
				if (points_bb.maxx):
					self.maxx = points_bb.maxx

				if (points_bb.miny):
					self.miny = points_bb.miny

				if (points_bb.maxy):
					self.maxy = points_bb.maxy

				if (points_bb.maxz):
					self.maxz = points_bb.maxz

			self.whole_body.move_to_go()
			self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			rospy.sleep(self.delay)

			##################
                        #Find closest point
			counter = 0
			while True:
				#Call service with parameter (maxx, miny, maxy, maxz)
				closest_point = self.get_closest_point(self.maxx, self.miny, self.maxy, self.maxz).closest_point

				counter += 1
				if (closest_point.ispoint == True):
					if closest_point.metric_point[0] > 0.0:
						break

				elif (counter == 2):
					break

			if (closest_point.ispoint == False):
				return 'unknown'

			#Print information out
			p = (closest_point.metric_point[0], closest_point.metric_point[1], closest_point.metric_point[2] )
			rospy.loginfo('closestpoint.base_footprint_point={}'.format(p))

			#drift_z = 0.07 #0.125

			arm_lift = 0
			if 0.695 - (closest_point.metric_point[2] + self.drift_z) < 0.10:
				arm_lift = self.height_diff

			diff_x = closest_point.metric_point[0] - self.distance
			diff_y = closest_point.metric_point[1] - 0.040
			self.omni_base.go_rel(diff_x, diff_y, 0.0, 300.0)

			#self.whole_body.move_to_neutral()
			diff_z = 0.695 + arm_lift - (closest_point.metric_point[2] + self.drift_z)
			arm_flex = math.acos(1 - diff_z/0.345) #np.pi/3

			self.whole_body.move_to_joint_positions({'arm_flex_joint': -arm_flex,
							    'arm_lift_joint': arm_lift,#0.1382,
							    'arm_roll_joint': 0.,
							    'wrist_flex_joint': -np.pi/2+arm_flex,
							    'wrist_roll_joint': self.wrist_roll})

			#Convert pixel coordinates into metric coordinats in hand_palm_link reference system
			#x1 = closest_point.metric_point[2]
			#y1 = -1*closest_point.metric_point[1]
			#z1 = closest_point.metric_point[0]

			#self.whole_body.linear_weight = 100.0
			#diff_x = x1 - 0.685
			#self.whole_body.move_end_effector_pose(geometry.pose(x=diff_x, ek=-1.57), 'hand_palm_link')
			#self.whole_body.linear_weight = 100.0

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

#Attempts to move the base to a relative position to the closest point
#tilt: head tilt angle
#delay: to update camera buffer
#wrist_roll: hand rotation
#(miny,maxy): handle valid height region (negative values uses default yaml file configuration)
#position(x,y): relative position FROM the final robot position TO the closest point
class Align2Point(smach.State):
	def __init__(self, robot, delay = 1.0, tilt = -0.70, wrist_roll = 0.0, maxx = -1.0, miny = -1.0, maxy = -1.0, maxz = -1.0, point = (0.0, 0.0), tf_buffer = None, timeout = None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown', 'timeout'],
					   input_keys = ['fun_params', 'start_time', 'stop_time'])

		self.robot = robot
		#Connect to base
		#self.move_base_pub = rospy.Publisher('/hsrb/command_velocity', Twist)

                #Start LineFinder client
                rospy.wait_for_service('/erasers/navigation/front_point_srv')
                self.get_closest_point = rospy.ServiceProxy('/erasers/navigation/front_point_srv', GetClosestPoint)

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.omni_base = self.robot.get("omni_base")

		self.delay = delay
		self.tilt = tilt
		self.wrist_roll = wrist_roll

		self.maxx = maxx
		self.miny = miny
		self.maxy = maxy
		self.maxz = maxz

		self.point = point

	def execute(self, userdata):
		try:
			robot_pose = userdata.fun_params.robot_pose
			if (robot_pose.in_use):
				if (robot_pose.delay):
					self.delay = robot_pose.delay

				if (robot_pose.tilt):
					self.tilt = robot_pose.tilt

				if (robot_pose.wrist_roll):
					self.wrist_roll = robot_pose.wrist_roll

			points_bb = userdata.fun_params.points_bb
			if (points_bb.in_use):
				if (points_bb.maxx):
					self.maxx = points_bb.maxx

				if (points_bb.miny):
					self.miny = points_bb.miny

				if (points_bb.maxy):
					self.maxy = points_bb.maxy

				if (points_bb.maxz):
					self.maxz = points_bb.maxz

			self.whole_body.move_to_go()
			self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			rospy.sleep(self.delay)

			##################
                        #Find closest point
			counter = 0
			while True:
				#Call service with parameter (maxx, miny, maxy, maxz)
				closest_point = self.get_closest_point(self.maxx, self.miny, self.maxy, self.maxz).closest_point

				counter += 1
				if (closest_point.ispoint == True):
					if closest_point.metric_point[0] > 0.0:
						break

				elif (counter == 2):
					break

			if (closest_point.ispoint == False):
				return 'unknown'

			#Print information out
			p = (closest_point.metric_point[0], closest_point.metric_point[1], closest_point.metric_point[2] )
			rospy.loginfo('closestpoint.base_footprint_point={}'.format(p))

			diff_x = closest_point.metric_point[0] - self.point[0]
			diff_y = closest_point.metric_point[1] - self.point[1]

			#self.whole_body.move_end_effector_pose(((0.0, -1*diff_x, -1*diff_y), (0, 0, 0, 1)), 'hand_palm_link')
			self.omni_base.go_rel(diff_x, diff_y, 0.0, 300.0)

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

class Align2ExtremePoint(smach.State):
	def __init__(self, robot, delay = 1.0, tilt = -0.70, maxx = -1.0, miny = -1.0, maxy = -1.0, maxz = -1.0, point = (0.0, 0.0), forward = True, tf_buffer = None, timeout = None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown', 'timeout'],
					   input_keys = ['fun_params', 'rightside', 'start_time', 'stop_time'])

		self.robot = robot

                #Start LineFinder client
                rospy.wait_for_service('/erasers/navigation/front_line_srv')
                self.get_front_line = rospy.ServiceProxy('/erasers/navigation/front_line_srv', GetLinePoint)

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.omni_base = self.robot.get("omni_base")

		self.delay = delay
		self.tilt = tilt

		self.maxx = maxx
		self.miny = miny
		self.maxy = maxy
		self.maxz = maxz
	
		self.forward = forward

		self.point = point

	def execute(self, userdata):
		try:
			robot_pose = userdata.fun_params.robot_pose
			if (robot_pose.in_use):
				if (robot_pose.delay):
					self.delay = robot_pose.delay

				if (robot_pose.tilt):
					self.tilt = robot_pose.tilt

			points_bb = userdata.fun_params.points_bb
			if (points_bb.in_use):
				if (points_bb.maxx):
					self.maxx = points_bb.maxx

				if (points_bb.miny):
					self.miny = points_bb.miny

				if (points_bb.maxy):
					self.maxy = points_bb.maxy

				if (points_bb.maxz):
					self.maxz = points_bb.maxz

		        # initialize hsr pose
			if self.forward:
        			self.whole_body.move_to_go()

			self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			rospy.sleep(self.delay)

			##################
                        #Find closest line's starting and ending points
			counter = 0
			while True:
				#Call service with parameter (maxx, miny, maxy, maxz)
				line_point = self.get_front_line(self.maxx, self.miny, self.maxy, self.maxz).line_point
				#metric_start = self.xtion.from_pixel((line_point.pixel_start[0], line_point.pixel_start[1]), target_frame='base_footprint')
				#metric_end = self.xtion.from_pixel((line_point.pixel_end[0], line_point.pixel_end[1]), target_frame='base_footprint')
				#diff_point = abs((metric_start[0][0]+line_point.depth_start*metric_start[1][0]) - (metric_end[0][0]+line_point.depth_end*metric_end[1][0]))
				#Look for point in the same line
				#if ( line_point.depth_start != 0 and line_point.depth_end != 0 and diff_point < 0.25 ):
				#	break

				counter += 1
				if (line_point.isline == True):
					diff_point = abs(line_point.metric_start[0] - line_point.metric_end[0])
					if diff_point < 0.25:
						break

				elif (counter == 2):
					break

			if (line_point.isline == False):
				return 'unknown'

			#A point in robot coordinates is
			#x positive to the front
			#y positive to the left
			#z positive upwards
			x1 = line_point.metric_start[0]
			y1 = line_point.metric_start[1]
			z1 = line_point.metric_start[2]

			p1 = (x1, y1, z1)

			x2 = line_point.metric_end[0]
			y2 = line_point.metric_end[1]
			z2 = line_point.metric_end[2]

			p2 = (x2, y2, z2)

			if (userdata.rightside):
				if (y1 > y2):
					p = (x1, y1, z1)
				else:
					p = (x2, y2, z2)
			else:
				if (y1 < y2):
					p = (x1, y1, z1)
				else:
					p = (x2, y2, z2)

			#Print information out
			rospy.loginfo('closestpoint.base_footprint_point={}'.format(p))

			diff_x = p[0] - self.point[0]
			diff_y = p[1] - self.point[1]

			#self.whole_body.move_end_effector_pose(((0.0, -1*diff_x, -1*diff_y), (0, 0, 0, 1)), 'hand_palm_link')
			self.omni_base.go_rel(diff_x, diff_y, 0.0, 300.0)

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

#Determines whether the door is opened by considering its distance to the robot
#distance: from robot to opened door
#threshold: error from robot to opened door
#tilt: head tilt angle
#delay: to update camera buffer
#(miny,maxy): handle valid height region (negative values uses default yaml file configuration)
class CheckDepth(smach.State):
	def __init__(self, robot, delay = 1.0, distance = 0.80, threshold = 0.1, tilt = -0.70, maxx = -1.0, miny  = -1.0, maxy = -1.0, maxz = -1.0, tf_buffer = None, timeout = None):
		smach.State.__init__(self, outcomes=['object', 'space', 'unknown', 'failure'],
					   input_keys = ['fun_params', 'start_time', 'stop_time'])

		self.robot = robot
                #Start LineFinder client
                rospy.wait_for_service('/erasers/navigation/front_point_srv')
                self.get_closest_point = rospy.ServiceProxy('/erasers/navigation/front_point_srv', GetClosestPoint)

		#if tf_buffer:
		#	self.tf_buffer = tf_buffer
		#else:
		#	self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
		#	tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')

		self.delay = delay
		self.tilt = tilt
		self.distance = distance

		self.maxx = maxx
		self.miny = miny
		self.maxy = maxy
		self.maxz = maxz

		self.threshold = threshold

	def execute(self, userdata):
		try:
			robot_pose = userdata.fun_params.robot_pose
			if (robot_pose.in_use):
				if (robot_pose.delay):
					self.delay = robot_pose.delay

				if (robot_pose.tilt):
					self.tilt = robot_pose.tilt

				if (robot_pose.distance):
					self.distance = robot_pose.distance

			points_bb = userdata.fun_params.points_bb
			if (points_bb.in_use):
				if (points_bb.maxx):
					self.maxx = points_bb.maxx

				if (points_bb.miny):
					self.miny = points_bb.miny

				if (points_bb.maxy):
					self.maxy = points_bb.maxy

				if (points_bb.maxz):
					self.maxz = points_bb.maxz

			self.whole_body.move_to_go()
			self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			rospy.sleep(self.delay)

			##################
                        #Find closest point
			counter = 0
			while True:
				#Call service with parameter (maxx, miny, maxy, maxz)
				closest_point = self.get_closest_point(self.maxx, self.miny, self.maxy, self.maxz).closest_point

				counter += 1
				if (closest_point.ispoint == True):
					if closest_point.metric_point[0] > 0.0:
						break

				elif (counter == 2):
					break

			if (closest_point.ispoint == False):
				return 'unknown'

			#Print information out
			p = (closest_point.metric_point[0], closest_point.metric_point[1], closest_point.metric_point[2] )
			rospy.loginfo('closestpoint.base_footprint_point={}'.format(p))

			diff_depth = abs(closest_point.metric_point[0] - self.distance)
			if diff_depth < self.threshold:
				return 'object'
			else:
				return 'space'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'
