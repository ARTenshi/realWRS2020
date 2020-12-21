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

from geometry_msgs.msg import Twist

from common import HSRB_Xtion

import hsrb_interface

#LineFinder libraries
from erasers_nav_msgs.srv import GetLinePoint
from modules import FunctionParams

#Aligns the robot (camera frame) relative to the door plane at a given position and orientation
#position: distance from the robot to the closest plane
#orientation: between the camera frame and the closest plane
#(miny,maxy): valid height region (negative values uses default yaml file configuration)
class FindEdge(smach.State):
	def __init__(self, robot, delay = 1.0, tilt = -0.70, position = [], orientation = [], maxx = -1.0, miny = -1.0, maxy = -1.0, maxz = -1.0, tf_buffer = None, timeout = None, forward = True, align = True):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown', 'timeout'],
					   input_keys = ['fun_params', 'start_time', 'stop_time'])

		self.robot = robot
		#Connect to base
		self.move_base_pub = rospy.Publisher('/hsrb/command_velocity', Twist)

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

			twist = Twist()

			if self.orientation and abs(edge_angle-self.orientation) > 0.035:
				angle = edge_angle - self.orientation

				self.omni_base.go_rel(0.0, 0.0, angle, 300.0)

			if self.position and abs(edge_dist-self.position) > 0.025:
				distance = edge_dist - self.position

				self.omni_base.go_rel(distance, 0.0, 0.0, 300.0)

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'
