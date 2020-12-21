#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import smach
import actionlib
import tf2_ros
import math
import random
import numpy as np
import traceback
import cv2
import time

#Moveit! libraries
import moveit_commander
from geometry_msgs.msg import Vector3Stamped, Pose, PoseStamped, PointStamped
from geometry_msgs.msg import WrenchStamped, Twist


#LineFinder libraries
from erasers_nav_msgs.srv import GetObjectsCentroid
from erasers_nav_msgs.srv import GetClosestPoint
from erasers_nav_msgs.srv import GetSpaceCentroid
from erasers_nav_msgs.srv import GetLinePoint

# yolo_tf detection
import sys
import os
sys.path.append("/".join(os.path.abspath(os.path.dirname(__file__)).split("/")[:-1]))
from modules import FunctionParams

import matplotlib.pyplot as plt

from time import strftime, gmtime

_vel = 0.1
_twist = Twist()
_twist.linear.x = 0.0
_twist.linear.y = 0.0
_twist.angular.z = 0.0

IMAGE_SAVE_PATH = '/home/roboworks/catkin_ws/src/mapping/src/erasers_vision/data/'

#Aligns the robot (camera frame) relative to the door plane at a given position and orientation
#position: distance from the robot to the closest plane
#orientation: between the camera frame and the closest plane
#(miny,maxy): valid height region (negative values uses default yaml file configuration)
class FindEdge(smach.State):
	def __init__(self, robot, delay = 1.0, tilt = -0.70, position = [], orientation = [], maxx = -1.0, miny = -1.0, maxy = -1.0, maxz = -1.0, tf_buffer = None, timeout = None, forward = True, align = True):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown', 'timeout'],
					   input_keys = ['fun_params', 'start_time', 'stop_time'])

		self.whole_body = robot.whole_body
		self.head = robot.head
		self.arm = robot.arm
		self.gripper = robot.gripper
		self.omni_base = robot.omni_base
		self.cmd_vel = robot.cmd_vel

                #Start LineFinder client
                rospy.wait_for_service('/erasers/navigation/front_line_srv')
                self.get_front_line = rospy.ServiceProxy('/erasers/navigation/front_line_srv', GetLinePoint)

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		self.delay = delay
		self.tilt = tilt

		if position:
			self.position = position
		else:
			self.position = []

		if orientation:
			self.orientation = orientation*3.1416/180
		else:
			self.orientation = []

		self.maxx = maxx
		self.miny = miny
		self.maxy = maxy
		self.maxz = maxz
	
		self.forward = forward
		self.align = align

	def execute(self, userdata):
		try:
			if not self.align:
        			return 'success'

			robot_pose = userdata.fun_params.robot_pose
			if (robot_pose.in_use):
				if (robot_pose.delay):
					self.delay = robot_pose.delay

				if (robot_pose.tilt):
					self.tilt = robot_pose.tilt

			edges_bb = userdata.fun_params.edges_bb
			if (edges_bb.in_use):
				if (edges_bb.position):
					self.position = edges_bb.position

				if (edges_bb.orientation):
					self.orientation = edges_bb.orientation*3.1416/180

				if (edges_bb.maxx):
					self.maxx = edges_bb.maxx

				if (edges_bb.miny):
					self.miny = edges_bb.miny

				if (edges_bb.maxy):
					self.maxy = edges_bb.maxy

				if (edges_bb.maxz):
					self.maxz = edges_bb.maxz

		        # initialize hsr pose
			if self.forward:
        			#self.whole_body.move_to_go()
				self.arm.stop()
				self.arm.set_named_target('go')
				self.arm.go(wait=True)
				self.arm.stop()

			#self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			self.arm.stop()
			self.head.set_joint_value_target("head_tilt_joint", self.tilt)
			self.head.go(wait=True)
			self.arm.stop()

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

			#Convert pixel coordinates into metric coordinats in base_footprint reference system
			#metric_start = self.xtion.from_pixel((line_point.pixel_start[0], line_point.pixel_start[1]), target_frame='base_footprint')
			#x1 = metric_start[0][0]+line_point.depth_start*metric_start[1][0]
			#y1 = metric_start[0][1]+line_point.depth_start*metric_start[1][1]
			#z1 = metric_start[0][2]+line_point.depth_start*metric_start[1][2]

			#p1 = (x1, y1, z1)

                        #metric_end = self.xtion.from_pixel((line_point.pixel_end[0], line_point.pixel_end[1]), target_frame='base_footprint')
			#x2 = metric_end[0][0]+line_point.depth_end*metric_end[1][0]
			#y2 = metric_end[0][1]+line_point.depth_end*metric_end[1][1]
			#z2 = metric_end[0][2]+line_point.depth_end*metric_end[1][2]

			#p2 = (x2, y2, z2)

			x1 = line_point.metric_start[0]
			y1 = line_point.metric_start[1]
			z1 = line_point.metric_start[2]

			p1 = (x1, y1, z1)

			x2 = line_point.metric_end[0]
			y2 = line_point.metric_end[1]
			z2 = line_point.metric_end[2]

			p2 = (x2, y2, z2)

			#Calculate edge orientation with respect to the camera frame 
			# u = (u1, u2) = (x2-x1 , y2-y1), edge as a vector
			# v = (v1, v2) = (0, -1), horizontal vector with respect to camera frame
			# angle = arccos( (<u,v> / <u,u><v,v>)  ) = arccos ( -(y2-y1) / <x2-x1, y2-y1> )
			# Orientation is based on: sign(u1v2 - u2v1) = -u1 
			# sign < 0, less than pi radians
			# sign > 0, more than pi radians
			# sign = 0, opposite directions
			u1 = x2 - x1
			u2 = y2 - y1

			edge_angle = math.acos(-1*u2 / math.sqrt(pow(u1,2) + pow(u2,2)))
			if u1 < 0:
				edge_angle = -1*edge_angle

			#Get metric information
			edge_height = (z1 + z2)/2
			edge_dist = (x1 + x2)/2

			#Print information out
			rospy.loginfo('frontline.base_footprint_start={}'.format(p1))
			rospy.loginfo('frontline.base_footprint_end={}'.format(p2))
			rospy.loginfo('frontline.base_footprint_angle_degrees={}'.format(edge_angle*180/3.1416))
			rospy.loginfo('frontline.base_footprint_distance={}'.format((edge_dist)))
			rospy.loginfo('frontline.base_footprint_height={}'.format((edge_height)))

			if self.orientation and abs(edge_angle-self.orientation) > 0.035:
				angle = edge_angle - self.orientation
				#self.omni_base.go_rel(0.0, 0.0, angle, 300.0)
				self.omni_base.go_rel(0, 0., angle)

			if self.position and abs(edge_dist-self.position) > 0.025:
				distance = edge_dist - self.position
				#self.omni_base.go_rel(distance, 0.0, 0.0, 300.0)
				self.omni_base.go_rel(distance, 0., 0.)

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

class DetectObjectOnPlane(smach.State):
	def __init__(self, robot, delay = 1.0, tilt = -0.70, distance = 0.80, height_diff = 0.30, wrist_roll = 0.0, depth_min = 0.40, depth_max = 1.20, width = 0.80, height_min = 0.40, height_max = 0.80, min_area = 250, max_area = 100000, plane = True, bigplane = True, vertical = False, target = 0.0, tf_buffer = None, timeout = None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown', 'timeout'],
					   input_keys = ['fun_params', 'hsr', 'robot_info', 'start_time', 'stop_time'],
			    		   output_keys=['obj_name', 'obj_plane'])

		self.whole_body = robot.whole_body
		self.head = robot.head
		self.arm = robot.arm
		self.gripper = robot.gripper
		self.omni_base = robot.omni_base
		self.cmd_vel = robot.cmd_vel

                #Start ObjectFinder client
                rospy.wait_for_service('/erasers/navigation/object_finder_srv')
                self.get_objects_centroid = rospy.ServiceProxy('/erasers/navigation/object_finder_srv', GetObjectsCentroid)

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		self.delay = delay
		self.tilt = tilt
		self.distance = distance
		self.height_diff = height_diff
		self.wrist_roll = wrist_roll

		self.depth_min = depth_min
		self.depth_max = depth_max
		self.width = width
		self.height_min = height_min
		self.height_max = height_max

		self.min_area = min_area
		self.max_area = max_area
		self.plane = plane
		self.bigplane = bigplane
		self.vertical = vertical

		self.target = target

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

				if (robot_pose.height_diff):
					self.height_diff = robot_pose.height_diff

				if (robot_pose.wrist_roll):
					self.wrist_roll = robot_pose.wrist_roll

			objects_bb = userdata.fun_params.objects_bb
			if (objects_bb.in_use):
				if (objects_bb.depth_min):
					self.depth_min = objects_bb.depth_min

				if (objects_bb.depth_max):
					self.depth_max = objects_bb.depth_max

				if (objects_bb.width):
					self.width = objects_bb.width

				if (objects_bb.height_min):
					self.height_min = objects_bb.height_min

				if (objects_bb.height_max):
					self.height_max = objects_bb.height_max


			plane_info = userdata.fun_params.plane_info
			if (plane_info.in_use):
				if (plane_info.min_area):
					self.min_area = plane_info.min_area

				if (plane_info.max_area):
					self.max_area = plane_info.max_area

				if (plane_info.plane):
					self.plane = plane_info.plane

				if (plane_info.bigplane):
					self.bigplane = plane_info.bigplane

				if (plane_info.vertical):
					self.vertical = plane_info.vertical

			#self.whole_body.move_to_go()
			self.arm.stop()
			self.arm.set_named_target('go')
			self.arm.go(wait=True)
			self.arm.stop()

			#self.gripper.apply_force(1.0)
			self.gripper.stop()
			self.gripper.set_joint_value_target("hand_motor_joint", 0.)
			self.gripper.go(wait=True)
			self.gripper.stop()
		
			if 1.0 - self.height_min < 0.40:
				#self.whole_body.move_to_joint_positions({'arm_lift_joint': 0.15})
				self.arm.stop()
				self.arm.set_joint_value_target("arm_lift_joint", 0.15)
				self.arm.go(wait=True)
				self.arm.stop()
				self.arm.clear_pose_targets()


			
			#self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			self.arm.stop()
			self.head.set_joint_value_target("head_tilt_joint", self.tilt)
			self.head.go(wait=True)
			self.arm.stop()

			depth_min = self.depth_min
			depth_max = self.depth_max
			width_min = -1*self.width/2
			width_max = self.width/2
			height_min = self.height_min
			height_max = self.height_max
			min_area = self.min_area
			max_area = self.max_area

			##################
                        #Find closest object centroid
			counter = 0
			while True:
				#Call service with parameter (depth_min, depth_max, width_min, width_max, height_min, height_max, plane)
				#Output in camera coordinates WRT the robot
				#i_width positive to the right
				#j_height positive upwards
				#k_depth positive to the front
				#centroid = [i_width, j_height, k_depth]
				objects = self.get_objects_centroid(depth_min, depth_max, width_min, width_max, height_min, height_max, min_area, max_area, self.plane, self.bigplane, self.vertical).objects

				counter += 1
				if objects.isobject == True or counter == 2:
					break

			if objects.isobject == False:
				return 'unknown'

			#Get closest object index
			idx = -1
			n = objects.n
			dist = 10000000
			for i in range(objects.n):
				_dist = math.sqrt(math.pow(objects.centroid[i], 2) + math.pow(objects.centroid[2*n+i], 2))
				if (_dist < dist):
					dist = _dist
					idx = i 
			############
			############

			#A point in robot coordinates is
			#x positive to the front
			#y positive to the left
			#z positive upwards
			p = [objects.centroid[2*n+idx], -1*objects.centroid[idx], objects.centroid[n+idx] ]
			ev = [objects.eigenvectors[2*3*n+3*idx], -1*objects.eigenvectors[3*idx], objects.eigenvectors[3*n+3*idx] ]
			rospy.loginfo('closestobject.base_footprint_point={}'.format(p))

			#Object's centroid to Plane Distance
			userdata.obj_plane = p[2] - objects.plane_point[1]

			#TODO: Eigenvector alignment
			front_grasping = True
			if (abs(ev[0]) > abs(ev[2]) or abs(ev[1]) > abs(ev[2])):
				front_grasping = False

			height_min = objects.plane_point[1] + 0.02
			height_max = objects.plane_point[1] + (self.height_max - self.height_min) - 0.02

			if (p[2] - self.height_min < 0.05):
				p[2] = self.height_min + 0.05

			arm_lift = 0
			if 0.695 - p[2] < 0.10:
				arm_lift = self.height_diff

			if (userdata.hsr == 46):
				delta_depth = 0.30 #HSR46
			elif (userdata.hsr == 80):
				delta_depth = 0.20 #HSR80
			else:
				delta_depth = 0.20 #HSRSIM

			if p[0] - self.distance > delta_depth:
				arm_lift = arm_lift + 0.10

			#self.whole_body.move_to_neutral()
			self.arm.stop()
			self.arm.set_named_target('neutral')
			self.arm.go(wait=True)
			self.arm.stop()

			if (userdata.hsr == 46):
				drift_z = 0.02 #HSR46
			elif (userdata.hsr == 80):
				drift_z = 0.00 #HSR80
			else:
				drift_z = 0.00 #HSRSIM
			
			diff_z = 0.695 + arm_lift - (p[2] + drift_z)
			arm_flex = math.acos(1 - diff_z/0.345) #np.pi/3

			#self.gripper.command(0.9)
			self.gripper.stop()
			self.gripper.set_joint_value_target("hand_motor_joint", 1.)
			self.gripper.go(wait=True)
			self.gripper.stop()

			if (userdata.hsr == 46):
				delta_dist = 0.00 #HSR46
			elif (userdata.hsr == 80):
				delta_dist = 0.02 #HSR80
			else:
				delta_dist = 0.00 #HSRSIM
			
			diff_x = p[0] + delta_dist - self.distance

			if (userdata.hsr == 46):
				diff_y = p[1] - 0.02 #HSR46
			elif (userdata.hsr == 80):
				diff_y = p[1] - 0.05 #HSR80
			else:
				diff_y = p[1] - 0.05 #HSRSIM

			rospy.logwarn(diff_x)
			rospy.logwarn(diff_y)

			#self.whole_body.move_end_effector_pose(((0, -1*diff_y, 0), (0, 0, 0, 1)), 'hand_palm_link')
			#self.omni_base.go_rel(0., 0., 1.57)
			#self.omni_base.go_rel(diff_y, 0., 0.)
			#self.omni_base.go_rel(0., 0., -1.57)
			if diff_y < 0:
				_twist.linear.y = -1*_vel
			else:
				_twist.linear.y = _vel
			_time = abs(diff_y / _vel)
			start_time = rospy.Time.now().to_sec()
			while rospy.Time.now().to_sec() - start_time < _time:
				self.cmd_vel.publish(_twist)
			_twist.linear.y = 0.0
			self.cmd_vel.publish(_twist)

			self.arm.stop()
			target_joints = {'arm_flex_joint': -arm_flex,
					 'arm_lift_joint': arm_lift,
					 'arm_roll_joint': 0.0,
					 'head_pan_joint': 0.0,
					 'head_tilt_joint': -0.76,
					 'wrist_flex_joint': -np.pi/2+arm_flex,
					 'wrist_roll_joint': self.wrist_roll}
			self.arm.go(target_joints,wait=True)
			self.arm.stop()
			self.arm.clear_pose_targets()

			#self.whole_body.move_end_effector_pose(((0, 0, diff_x), (0, 0, 0, 1)), 'hand_palm_link')
			self.omni_base.go_rel(diff_x, 0., 0.)

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			#Grasp Visual
			rospy.loginfo('arm_distance: {}'.format( 0.345*math.sin(abs(arm_flex)) ) )
			gripper2target_distance = self.distance - 0.28
			gripper2target_distance = gripper2target_distance - self.target
			diff_dist = gripper2target_distance - 0.345*math.sin( abs(arm_flex) )

			#self.whole_body.move_end_effector_pose(((0, 0, diff_dist), (0, 0, 0, 1)), 'hand_palm_link')
			self.omni_base.go_rel(diff_dist, 0., 0.)

			rospy.sleep(2)

			#self.gripper.apply_force(1.0)
			self.gripper.stop()
			self.gripper.set_joint_value_target("hand_motor_joint", 0.)
			self.gripper.go(wait=True)
			self.gripper.stop()

			rospy.sleep(3)

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

class DetectObjectOnVerticalPlane(smach.State):
	def __init__(self, robot, delay = 1.0, tilt = -0.70, distance = 0.80, height_diff=0.30, wrist_roll = None, depth_min = 0.40, depth_max = 1.20, width = 0.80, height_min = 0.40, height_max = 0.80, min_area = 250, max_area = 100000, plane = True, bigplane = True, vertical = True, target = 0.0, tf_buffer=None, timeout=None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown', 'timeout'],
					   input_keys = ['fun_params', 'hsr', 'robot_info', 'start_time', 'stop_time'],
			    		   output_keys=['obj_name'])

		self.whole_body = robot.whole_body
		self.head = robot.head
		self.arm = robot.arm
		self.gripper = robot.gripper
		self.omni_base = robot.omni_base
		self.cmd_vel = robot.cmd_vel

                #Start ObjectFinder client
                rospy.wait_for_service('/erasers/navigation/object_finder_srv')
                self.get_objects_centroid = rospy.ServiceProxy('/erasers/navigation/object_finder_srv', GetObjectsCentroid)

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		self.delay = delay
		self.tilt = tilt
		self.distance = distance
		self.height_diff = height_diff
		self.wrist_roll = wrist_roll

		self.depth_min = depth_min
		self.depth_max = depth_max
		self.width = width
		self.height_min = height_min
		self.height_max = height_max

		self.min_area = min_area
		self.max_area = max_area
		self.plane = plane
		self.bigplane = bigplane
		self.vertical = vertical

		self.target = target

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

				if (robot_pose.height_diff):
					self.height_diff = robot_pose.height_diff

				if (robot_pose.wrist_roll):
					self.wrist_roll = robot_pose.wrist_roll

			objects_bb = userdata.fun_params.objects_bb
			if (objects_bb.in_use):
				if (objects_bb.depth_min):
					self.depth_min = objects_bb.depth_min

				if (objects_bb.depth_max):
					self.depth_max = objects_bb.depth_max

				if (objects_bb.width):
					self.width = objects_bb.width

				if (objects_bb.height_min):
					self.height_min = objects_bb.height_min

				if (objects_bb.height_max):
					self.height_max = objects_bb.height_max


			plane_info = userdata.fun_params.plane_info
			if (plane_info.in_use):
				if (plane_info.min_area):
					self.min_area = plane_info.min_area

				if (plane_info.max_area):
					self.max_area = plane_info.max_area

				if (plane_info.plane):
					self.plane = plane_info.plane

				if (plane_info.bigplane):
					self.bigplane = plane_info.bigplane

				if (plane_info.vertical):
					self.vertical = plane_info.vertical

			#self.whole_body.move_to_go()
			self.arm.stop()
			self.arm.set_named_target('go')
			self.arm.go(wait=True)
			self.arm.stop()

			#self.gripper.apply_force(1.0)
			self.gripper.stop()
			self.gripper.set_joint_value_target("hand_motor_joint", 0.)
			self.gripper.go(wait=True)
			self.gripper.stop()
		
			if 1.0 - self.height_min < 0.40:
				#self.whole_body.move_to_joint_positions({'arm_lift_joint': 0.15})
				self.arm.stop()
				self.arm.set_joint_value_target("arm_lift_joint", 0.15)
				self.arm.go(wait=True)
				self.arm.stop()
				self.arm.clear_pose_targets()

			#self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			self.arm.stop()
			self.head.set_joint_value_target("head_tilt_joint", self.tilt)
			self.head.go(wait=True)
			self.arm.stop()

			depth_min = self.depth_min
			depth_max = self.depth_max
			width_min = -1*self.width/2
			width_max = self.width/2
			height_min = self.height_min
			height_max = self.height_max
			min_area = self.min_area
			max_area = self.max_area

			##################
                        #Find closest object centroid
			counter = 0
			while True:
				#Call service with parameter (depth_min, depth_max, width_min, width_max, height_min, height_max, plane)
				#Output in camera coordinates WRT the robot
				#i_width positive to the right
				#j_height positive upwards
				#k_depth positive to the front
				#centroid = [i_width, j_height, k_depth]
				objects = self.get_objects_centroid(depth_min, depth_max, width_min, width_max, height_min, height_max, min_area, max_area, self.plane, self.bigplane, self.vertical).objects

				counter += 1
				if objects.isobject == True or counter == 2:
					break

			if objects.isobject == False:
				return 'unknown'

			#Get closest object index
			idx = -1
			n = objects.n
			dist = 10000000
			for i in range(objects.n):
				_dist = math.sqrt(math.pow(objects.centroid[i], 2) + math.pow(objects.centroid[2*n+i], 2))
				if (_dist < dist):
					dist = _dist
					idx = i 
			############
			############

			#A point in robot coordinates is
			#x positive to the front
			#y positive to the left
			#z positive upwards
			p = [objects.centroid[2*n+idx], -1*objects.centroid[idx], objects.centroid[n+idx] ]
			#Plane to Robot depth correction
			p[0] = p[0] - (objects.plane_point[2] - self.distance)
			ev = [objects.eigenvectors[2*3*n+3*idx], -1*objects.eigenvectors[3*idx], objects.eigenvectors[3*n+3*idx] ]
			rospy.loginfo('closestobject.base_footprint_point={}'.format(p))

			#TODO: Eigenvector alignment
			if (self.wrist_roll == None):
				self.wrist_roll = 0.0
				if (abs(ev[0]) > abs(ev[2]) or abs(ev[1]) > abs(ev[2])):
					self.wrist_roll = -1.57

			arm_lift = 0

			if (userdata.hsr == 46):
				drift_z = 0.04 #HSR46
			elif (userdata.hsr == 80):
				drift_z = 0.01 #HSR80
			else:
				drift_z = 0.04 #HSRSIM

			if 0.695 - (p[2] - drift_z) < 0.10:
				arm_lift = self.height_diff

			#self.whole_body.move_to_neutral()
			self.arm.stop()
			self.arm.set_named_target('neutral')
			self.arm.go(wait=True)
			self.arm.stop()

			diff_z = 0.695 + arm_lift - (p[2] + drift_z)
			arm_flex = math.acos(1 - diff_z/0.345) #np.pi/3

			#self.gripper.command(0.9)
			self.gripper.stop()
			self.gripper.set_joint_value_target("hand_motor_joint", 1.)
			self.gripper.go(wait=True)
			self.gripper.stop()

			if (userdata.hsr == 46):
				delta_dist = 0.00 #HSR46
			elif (userdata.hsr == 80):
				delta_dist = 0.00 #HSR80
			else:
				delta_dist = 0.00 #HSRSIM

			diff_x = p[0] + delta_dist - self.distance 

			if (userdata.hsr == 46):
				diff_y = p[1] - 0.02 #HSR46
			elif (userdata.hsr == 80):
				diff_y = p[1] - 0.04 #HSR80
			else:
				diff_y = p[1] - 0.04 #HSRSIM

			rospy.logwarn(diff_x)
			rospy.logwarn(diff_y)

			#self.whole_body.move_end_effector_pose(((0, -1*diff_y, 0), (0, 0, 0, 1)), 'hand_palm_link')
			#self.omni_base.go_rel(0., 0., 1.57)
			#self.omni_base.go_rel(diff_y, 0., 0.)
			#self.omni_base.go_rel(0., 0., -1.57)
			if diff_y < 0:
				_twist.linear.y = -1*_vel
			else:
				_twist.linear.y = _vel
			_time = abs(diff_y / _vel)
			start_time = rospy.Time.now().to_sec()
			while rospy.Time.now().to_sec() - start_time < _time:
				self.cmd_vel.publish(_twist)
			_twist.linear.y = 0.0
			self.cmd_vel.publish(_twist)

			self.arm.stop()
			target_joints = {'arm_flex_joint': -arm_flex,
					 'arm_lift_joint': arm_lift,
					 'arm_roll_joint': 0.0,
					 'head_pan_joint': 0.0,
					 'head_tilt_joint': -0.76,
					 'wrist_flex_joint': -np.pi/2+arm_flex,
					 'wrist_roll_joint': self.wrist_roll}
			self.arm.go(target_joints,wait=True)
			self.arm.stop()
			self.arm.clear_pose_targets()

			#self.whole_body.move_end_effector_pose(((0, 0, diff_x), (0, 0, 0, 1)), 'hand_palm_link')
			self.omni_base.go_rel(diff_x, 0., 0.)

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			#Grasp Visual
			rospy.loginfo('arm_distance: {}'.format( 0.345*math.sin(abs(arm_flex)) ) )
			gripper2target_distance = self.distance - 0.28
			gripper2target_distance = gripper2target_distance - self.target
			diff_dist = gripper2target_distance - 0.345*math.sin( abs(arm_flex) )

			#self.whole_body.move_end_effector_pose(((0, 0, diff_dist), (0, 0, 0, 1)), 'hand_palm_link')
			self.omni_base.go_rel(diff_dist, 0., 0.)

			#self.gripper.apply_force(1.0)
			self.gripper.stop()
			self.gripper.set_joint_value_target("hand_motor_joint", 0.)
			self.gripper.go(wait=True)
			self.gripper.stop()

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'


class DetectGraspObjectOnFloor(smach.State):
	def __init__(self, robot, delay = 1.0, tilt = -0.96, height_diff = 0.30, wrist_roll = 0.0, depth_min = 0.20, depth_max = 1.20, width = 1.20, height_min = 0.0, height_max = 0.40, min_area = 250, max_area = 100000, plane = True, bigplane = False, vertical = False, tf_buffer=None, timeout=None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown', 'timeout'],
					   input_keys = ['fun_params', 'hsr', 'robot_info', 'start_time', 'stop_time'],
			    		   output_keys=['obj_name'])

		self.whole_body = robot.whole_body
		self.head = robot.head
		self.arm = robot.arm
		self.gripper = robot.gripper
		self.omni_base = robot.omni_base
		self.cmd_vel = robot.cmd_vel

                #Start ObjectFinder client
                rospy.wait_for_service('/erasers/navigation/object_finder_srv')
                self.get_objects_centroid = rospy.ServiceProxy('/erasers/navigation/object_finder_srv', GetObjectsCentroid)

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		self.delay = delay
		self.tilt = tilt
		self.height_diff = height_diff
		self.wrist_roll = wrist_roll

		self.depth_min = depth_min
		self.depth_max = depth_max
		self.width = width
		self.height_min = height_min
		self.height_max = height_max

		self.min_area = min_area
		self.max_area = max_area
		self.plane = plane
		self.bigplane = bigplane
		self.vertical = vertical

	def execute(self, userdata):
		try:
			robot_pose = userdata.fun_params.robot_pose
			if (robot_pose.in_use):
				if (robot_pose.delay):
					self.delay = robot_pose.delay

				if (robot_pose.tilt):
					self.tilt = robot_pose.tilt

				if (robot_pose.height_diff):
					self.height_diff = robot_pose.height_diff

				if (robot_pose.wrist_roll):
					self.wrist_roll = robot_pose.wrist_roll

			objects_bb = userdata.fun_params.objects_bb
			if (objects_bb.in_use):
				if (objects_bb.depth_min):
					self.depth_min = objects_bb.depth_min

				if (objects_bb.depth_max):
					self.depth_max = objects_bb.depth_max

				if (objects_bb.width):
					self.width = objects_bb.width

				if (objects_bb.height_min):
					self.height_min = objects_bb.height_min

				if (objects_bb.height_max):
					self.height_max = objects_bb.height_max


			plane_info = userdata.fun_params.plane_info
			if (plane_info.in_use):
				if (plane_info.min_area):
					self.min_area = plane_info.min_area

				if (plane_info.max_area):
					self.max_area = plane_info.max_area

				if (plane_info.plane):
					self.plane = plane_info.plane

				if (plane_info.bigplane):
					self.bigplane = plane_info.bigplane

				if (plane_info.vertical):
					self.vertical = plane_info.vertical

			#self.whole_body.move_to_go()
			self.arm.stop()
			self.arm.set_named_target('go')
			self.arm.go(wait=True)
			self.arm.stop()

			#self.gripper.apply_force(1.0)
			self.gripper.stop()
			self.gripper.set_joint_value_target("hand_motor_joint", 0.)
			self.gripper.go(wait=True)
			self.gripper.stop()

			#self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			self.arm.stop()
			self.head.set_joint_value_target("head_tilt_joint", self.tilt)
			self.head.go(wait=True)
			self.arm.stop()

			depth_min = self.depth_min
			depth_max = self.depth_max
			width_min = -1*self.width/2
			width_max = self.width/2
			height_min = self.height_min
			height_max = self.height_max
			min_area = self.min_area
			max_area = self.max_area

			##################
                        #Find closest object centroid
			counter = 0
			while True:
				#Call service with parameter (depth_min, depth_max, width_min, width_max, height_min, height_max, plane)
				#Output in camera coordinates WRT the robot
				#i_width positive to the right
				#j_height positive upwards
				#k_depth positive to the front
				#centroid = [i_width, j_height, k_depth]
				objects = self.get_objects_centroid(depth_min, depth_max, width_min, width_max, height_min, height_max, min_area, max_area, self.plane, self.bigplane, self.vertical).objects

				counter += 1
				if objects.isobject == True or counter == 2:
					break

			if objects.isobject == False:
				return 'unknown'

			#Get closest object index
			idx = -1
			n = objects.n
			dist = 10000000
			for i in range(objects.n):
				_dist = math.sqrt(math.pow(objects.centroid[i], 2) + math.pow(objects.centroid[2*n+i], 2))
				if (_dist < dist):
					dist = _dist
					idx = i
			############
			############

			#A point in robot coordinates is
			#x positive to the front
			#y positive to the left
			#z positive upwards
			p = [objects.centroid[2*n+idx], -1*objects.centroid[idx], objects.centroid[n+idx] ]
			ev = [objects.eigenvectors[2*3*n+3*idx], -1*objects.eigenvectors[3*idx], objects.eigenvectors[3*n+3*idx] ]
			rospy.loginfo('closestobject.base_footprint_point={}'.format(p))
			rospy.loginfo('closestobject.base_footprint_ev={}'.format(ev))

			front_grasping = True
			if (abs(ev[0]) > 1.2*abs(ev[2]) or abs(ev[1]) > 1.2*abs(ev[2])):
				front_grasping = False

			#self.whole_body.move_to_neutral()
			self.arm.stop()
			self.arm.set_named_target('neutral')
			self.arm.go(wait=True)
			self.arm.stop()

			#self.gripper.command(1.2)
			self.gripper.stop()
			self.gripper.set_joint_value_target("hand_motor_joint", 1.)
			self.gripper.go(wait=True)
			self.gripper.stop()

			if (front_grasping):
				if (userdata.hsr == 46):
					delta_dist = 0.04 #HSR46
				elif (userdata.hsr == 80):
					delta_dist = 0.02 #HSR80
				else:
					delta_dist = 0.11 #HSRSIM

				arm_length = 0.40
				diff_x = p[0] - arm_length - delta_dist

				diff_y = p[1] - 0.040
				if ( p[1] < 0.15 ):
					if (userdata.hsr == 46):
						diff_y = p[1] - 0.030 #HSR46
					elif (userdata.hsr == 80):
						diff_y = p[1] - 0.050 #HSR80
					else:
						diff_y = p[1] - 0.050 #HSRSIM

				if ( p[1] < 0.0 ):
					if (userdata.hsr == 46):
						diff_y = p[1] - 0.015 #HSR46
					elif (userdata.hsr == 80):
						diff_y = p[1] - 0.050 #HSR80
					else:
						diff_y = p[1] - 0.050 #HSRSIM

				#self.whole_body.move_end_effector_pose(((0, -1*diff_y, 0), (0, 0, 0, 1)), 'hand_palm_link')
				#self.omni_base.go_rel(0., 0., 1.57)
				#self.omni_base.go_rel(diff_y, 0., 0.)
				#self.omni_base.go_rel(0., 0., -1.57)
				if diff_y < 0:
					_twist.linear.y = -1*_vel
				else:
					_twist.linear.y = _vel
				_time = abs(diff_y / _vel)
				start_time = rospy.Time.now().to_sec()
				while rospy.Time.now().to_sec() - start_time < _time:
					self.cmd_vel.publish(_twist)
				_twist.linear.y = 0.0
				self.cmd_vel.publish(_twist)

				arm_lift = 0
				if 0.695 - p[2] < 0.10:
					arm_lift = self.height_diff

				if (p[2] - self.height_min < 0.07):
					#self.whole_body.move_to_neutral()
					self.arm.stop()
					self.arm.set_named_target('neutral')
					self.arm.go(wait=True)
					self.arm.stop()

					#self.whole_body.move_end_effector_pose(((0, 0, 0.60*diff_x), (0, 0, 0, 1)), 'hand_palm_link')
					self.omni_base.go_rel(0.60*diff_x, 0., 0.)

					self.arm.stop()
					target_joints = {'arm_flex_joint': -2.45,
							 'arm_lift_joint': 0.0,
							 'arm_roll_joint': 0.0,
							 'head_pan_joint': 0.0,
							 'head_tilt_joint': -0.76,
							 'wrist_flex_joint': 0.88,
							 'wrist_roll_joint': 0.0}
					self.arm.go(target_joints,wait=True)
					self.arm.stop()
					self.arm.clear_pose_targets()

					#self.whole_body.move_end_effector_pose(((0, 0, 0.40*diff_x), (0, 0, 0, 1)), 'hand_palm_link')
					self.omni_base.go_rel(0.40*diff_x, 0., 0.)

					#To grasp from the base, uncomment these lines
					#note: arm_flex_joint+wrist_flex_joint=1.57
					self.arm.stop()
					target_joints = {'arm_flex_joint': -2.45,
							 'arm_lift_joint': 0.0,
							 'arm_roll_joint': 0.0,
							 'head_pan_joint': 0.0,
							 'head_tilt_joint': -0.76,
							 'wrist_flex_joint': 0.75,
							 'wrist_roll_joint': 0.0}
					self.arm.go(target_joints,wait=True)
					self.arm.stop()
					self.arm.clear_pose_targets()


				else:
					#self.whole_body.move_to_neutral()
					self.arm.stop()
					self.arm.set_named_target('neutral')
					self.arm.go(wait=True)
					self.arm.stop()

					#self.whole_body.move_end_effector_pose(((0, 0, 0.20*diff_x), (0, 0, 0, 1)), 'hand_palm_link')
					self.omni_base.go_rel(0.20*diff_x, 0., 0.)

					#p[2] = self.height_min + 0.05
					if (userdata.hsr == 46):
						drift_z = 0.03 #HSR46
					elif (userdata.hsr == 80):
						drift_z = -0.01 #HSR80
					else:
						drift_z = -0.05 #HSRSIM

					diff_z = 0.695 + arm_lift - (p[2] + drift_z)
					arm_flex = math.acos(1 - diff_z/0.345) #np.pi/3

					self.arm.stop()
					target_joints = {'arm_flex_joint': -arm_flex,
							 'arm_lift_joint': arm_lift,
							 'arm_roll_joint': 0.0,
							 'head_pan_joint': 0.0,
							 'head_tilt_joint': -0.76,
							 'wrist_flex_joint': -np.pi/2+arm_flex,
							 'wrist_roll_joint': 0.0}
					self.arm.go(target_joints,wait=True)
					self.arm.stop()
					self.arm.clear_pose_targets()

					#self.whole_body.move_end_effector_pose(((0, 0, 0.80*diff_x), (0, 0, 0, 1)), 'hand_palm_link')
					self.omni_base.go_rel(0.80*diff_x, 0., 0.)

			else:
				#x positive to the front
				#y positive to the left
				#z positive upwards
				norm_ev = math.sqrt( math.pow(ev[0], 2) + math.pow(ev[1], 2) + math.pow(ev[2], 2) ) # == 1.0

				#theta = arccos ( <u,v> / (||u|| ||v||) )
				theta = np.arccos(ev[0])

				if (theta > 1.57):
					theta = theta - 3.1416

				if (abs(theta) > 1.57):
					theta = 1.57

				rospy.loginfo('closestobject.theta={}'.format(theta))

				if (userdata.hsr == 46):
					delta_dist = -0.06 #HSR46
				elif (userdata.hsr == 80):
					delta_dist = -0.06 #HSR80
				else:
					delta_dist = -0.02 #HSRSIM

				arm_length = 0.40
				diff_x = p[0] - arm_length - delta_dist
				
				diff_y = p[1] - 0.040
				if ( p[1] < 0.15 ):
					if (userdata.hsr == 46):
						diff_y = p[1] - 0.030 #HSR46
					elif (userdata.hsr == 80):
						diff_y = p[1] - 0.050 #HSR80
					else:
						diff_y = p[1] - 0.050 #HSRSIM

				if ( p[1] < 0.0 ):
					if (userdata.hsr == 46):
						diff_y = p[1] - 0.015 #HSR46
					elif (userdata.hsr == 80):
						diff_y = p[1] - 0.035 #HSR80
					else:
						diff_y = p[1] - 0.050 #HSRSIM

				rospy.logwarn(diff_x)
				rospy.logwarn(diff_y)

				#self.whole_body.move_end_effector_pose(((0, -1*diff_y, 0), (0, 0, 0, 1)), 'hand_palm_link')
				#self.omni_base.go_rel(0., 0., 1.57)
				#self.omni_base.go_rel(diff_y, 0., 0.)
				#self.omni_base.go_rel(0., 0., -1.57)
				if diff_y < 0:
					_twist.linear.y = -1*_vel
				else:
					_twist.linear.y = _vel
				_time = abs(diff_y / _vel)
				start_time = rospy.Time.now().to_sec()
				while rospy.Time.now().to_sec() - start_time < _time:
					self.cmd_vel.publish(_twist)
				_twist.linear.y = 0.0
				self.cmd_vel.publish(_twist)

				#self.whole_body.move_end_effector_pose(((0, 0, diff_x), (0, 0, 0, 1)), 'hand_palm_link')
				self.omni_base.go_rel(diff_x, 0., 0.)

				arm_lift = 0
				if 0.695 - p[2] < 0.10:
					arm_lift = self.height_diff

				if (p[2] - self.height_min < 0.05):
					p[2] = self.height_min + 0.05

				#note: arm_flex_joint+wrist_flex_joint=3.14
				#self.whole_body.move_to_joint_positions({'wrist_roll_joint': theta})
				self.arm.stop()
				target_joints = {'arm_flex_joint': -1.57,
						 'arm_lift_joint': 0.0,
						 'arm_roll_joint': 0.0,
						 'head_pan_joint': 0.0,
						 'head_tilt_joint': -0.76,
						 'wrist_flex_joint': -1.57,
						 'wrist_roll_joint': theta}
				self.arm.go(target_joints,wait=True)
				self.arm.stop()
				self.arm.clear_pose_targets()

				self.arm.stop()
				target_joints = {'arm_flex_joint': -1.88,
						 'arm_lift_joint': 0.0,
						 'arm_roll_joint': 0.0,
						 'head_pan_joint': 0.0,
						 'head_tilt_joint': -0.76,
						 'wrist_flex_joint': -1.26,
						 'wrist_roll_joint': theta}
				self.arm.go(target_joints,wait=True)
				self.arm.stop()
				self.arm.clear_pose_targets()

			rospy.sleep(2)

			#self.gripper.apply_force(1.0)
			self.gripper.stop()
			self.gripper.set_joint_value_target("hand_motor_joint", 0.)
			self.gripper.go(wait=True)
			self.gripper.stop()

			rospy.sleep(4)

			#self.whole_body.move_to_go()
			self.arm.stop()
			self.arm.set_named_target('go')
			self.arm.go(wait=True)
			self.arm.stop()

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'
