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

import hsrb_interface
from hsrb_interface import geometry

#LineFinder libraries
from erasers_nav_msgs.srv import GetObjectsCentroid
from erasers_nav_msgs.srv import GetClosestPoint
from erasers_nav_msgs.srv import GetSpaceCentroid

# yolo_tf detection
import sys
import os
sys.path.append("/".join(os.path.abspath(os.path.dirname(__file__)).split("/")[:-1]))
from modules import MyDetection, FunctionParams

import matplotlib.pyplot as plt

from time import strftime, gmtime

IMAGE_SAVE_PATH = '/home/roboworks/src/mapping/src/erasers_vision/data/'

class DetectObject(smach.State):
	def __init__(self, robot, delay = 1.0, tilt = -0.70, distance = 0.80, height_diff = 0.30, wrist_roll = 0.0, depth = 1.20, width = 0.80, height_min = 0.40, height_max = 0.80, min_area = 250, max_area = 100000, plane = False, bigplane = False, vertical = False, tf_buffer = None, timeout = None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown', 'timeout'],
					   input_keys = ['fun_params', 'hsr', 'robot_info', 'start_time', 'stop_time'],
			    		   output_keys=['obj_name'])

		self.robot = robot
		#Connect to base
		#self.move_base_pub = rospy.Publisher('/hsrb/command_velocity', Twist)

                #Start PointFinder client
                rospy.wait_for_service('/erasers/navigation/front_point_srv')
                self.get_closest_point = rospy.ServiceProxy('/erasers/navigation/front_point_srv', GetClosestPoint)

                #Start ObjectFinder client
                rospy.wait_for_service('/erasers/navigation/object_finder_srv')
                self.get_objects_centroid = rospy.ServiceProxy('/erasers/navigation/object_finder_srv', GetObjectsCentroid)

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.omni_base = self.robot.get("omni_base")
		self.gripper = self.robot.get('gripper')

		self.delay = delay
		self.tilt = tilt
		self.distance = distance
		self.height_diff = height_diff
		self.wrist_roll = wrist_roll

		self.depth = depth
		self.width = width
		self.height_min = height_min
		self.height_max = height_max

		self.min_area = min_area
		self.max_area = max_area
		self.plane = plane
		self.bigplane = bigplane
		self.vertical = vertical

		# yolo detection class
		self.detecion = MyDetection()

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
				if (objects_bb.depth):
					self.depth = objects_bb.depth

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


			self.whole_body.move_to_go()
			self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			rospy.sleep(self.delay)

                        #Find object plane's information
			while True:
				#Call service with parameter (maxx, miny, maxy, maxz)
				closest_point = self.get_closest_point(0.20, self.height_min-0.05, self.height_min+0.05, 2.0).closest_point
				if closest_point.metric_point[0] > 0.0:
					break

			#Print information out
			cp = (closest_point.metric_point[0], closest_point.metric_point[1], closest_point.metric_point[2] )

			depth_min = cp[0] + 0.02
			depth_max = cp[0] + self.depth - 0.02
			width_min = -1*self.width/2
			width_max = self.width/2
			height_min = cp[2] + 0.02
			height_max = cp[2] + (self.height_max - self.height_min) - 0.02
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
			#yolo_tf detection
			out_img, color_img, depth_img, bb_list, obj_list = self.detecion.detection()
			rospy.logwarn("Objects detected: {}".format(obj_list))

			if (obj_list):
				# get minimum centroid comparing with yolo detection result
				centroid_center_x = objects.bbox[idx] + objects.bbox[2*n+idx]/2 # obj[i].x + obj[i].width/2
				centroid_center_y = objects.bbox[n+idx] + objects.bbox[3*n+idx]/2 # obj[i].y + obj[i].height/2
				centroid_center = np.array([centroid_center_x, centroid_center_y])

				prob = -1.0
				dist = 10000000 # initial value for comparison
				for i in range(len(bb_list)):
					_bbox = bb_list[i]

					_prob = -1.0
					for _obj in obj_list[i]:
						_prob = _obj[2]

					yolo_center_x = np.mean(_bbox[:2]) # mean[minX, maxX]
					yolo_center_y = np.mean(_bbox[2:]) # mean[minY, maxY]
					yolo_center = np.array([yolo_center_x, yolo_center_y])

					u = centroid_center - yolo_center
					_dist = np.linalg.norm(u)

					if _dist < dist and _prob > prob:
						dist = _dist
						prob = _prob
						idx_yolo = i

				rospy.logwarn("object: {}".format(obj_list[idx_yolo]))

				obj_name = None
				obj_prob = -1.0
				# compare yolo_BB and centroid.BB
				# TODO: exteralize config value (BB_DIST_THRESHOLD)
				BB_DIST_THRESHOLD = 300
				rospy.loginfo("The distance between yolo and pointcloud detection: {}, THRESHOLD: {} ".format(dist, BB_DIST_THRESHOLD)) 
				if dist > BB_DIST_THRESHOLD:
					rospy.loginfo("cannot recognize the object")
					obj_name = 'unknown'
				else:
					# chain detected label and ObjectList
					#rospy.logwarn(obj_list[idx_yolo])
					for _obj in obj_list[idx_yolo]:
						rospy.loginfo(_obj)
						if _obj[1].upper() in userdata.robot_info.obj_list.keys():
							obj_name = _obj[1]
							obj_prob = _obj[2]

				if obj_name is None:
					if len(obj_list) > 0:
						obj_name = 'unknown'
						obj_prob = -1.0
			else:
				obj_name = 'unknown'
				obj_prob = -1.0

			userdata.obj_name = obj_name.upper()
			rospy.logwarn("try to grasp {} {}".format(obj_name, obj_prob))

			# save image
			plt.imshow(out_img)
			now = strftime("%Y%m%d-%H%M%s", gmtime())
			plt.savefig(IMAGE_SAVE_PATH+"image_{}".format(now))
			############
			############

			#A point in robot coordinates is
			#x positive to the front
			#y positive to the left
			#z positive upwards
			p = [objects.centroid[2*n+idx], -1*objects.centroid[idx], objects.centroid[n+idx] ]
			rospy.loginfo('closestobject.base_footprint_point={}'.format(p))

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

			self.whole_body.move_to_neutral()

			if (userdata.hsr == 46):
				drift_z = 0.02 #HSR46
			elif (userdata.hsr == 80):
				drift_z = 0.00 #HSR80
			else:
				drift_z = 0.00 #HSRSIM
			
			diff_z = 0.695 + arm_lift - (p[2] + drift_z)
			arm_flex = math.acos(1 - diff_z/0.345) #np.pi/3

			self.whole_body.move_to_joint_positions({'arm_flex_joint': -arm_flex,
							    'arm_lift_joint': arm_lift,#0.1382,
							    'arm_roll_joint': 0.,
							    'wrist_flex_joint': -np.pi/2+arm_flex,
							    'wrist_roll_joint': self.wrist_roll})

			self.gripper.command(0.9)

			if (userdata.hsr == 46):
				delta_dist = 0.00 #HSR46
			elif (userdata.hsr == 80):
				delta_dist = 0.02 #HSR80
			else:
				delta_dist = 0.02 #HSRSIM
			
			diff_x = p[0] + delta_dist - self.distance

			if (userdata.hsr == 46):
				diff_y = p[1] - 0.02 #HSR46
			elif (userdata.hsr == 80):
				diff_y = p[1] - 0.06 #HSR80
			else:
				diff_y = p[1] - 0.06 #HSRSIM
			
			rospy.logwarn(diff_x)
			rospy.logwarn(diff_y)

			linear_weight = self.whole_body.linear_weight
			angular_weight = self.whole_body.angular_weight

			self.whole_body.linear_weight = 1
			self.whole_body.angular_weight = 100

			#self.omni_base.go_rel(diff_x, diff_y, 0.0, 300.0)
			self.whole_body.move_end_effector_pose(((0, -1*diff_y, 0), (0, 0, 0, 1)), 'hand_palm_link')
			self.whole_body.move_end_effector_pose(((0, 0, diff_x), (0, 0, 0, 1)), 'hand_palm_link')

			self.whole_body.linear_weight = linear_weight
			self.whole_body.angular_weight = angular_weight

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
	def __init__(self, robot, delay = 1.0, tilt = -0.70, distance = 0.80, height_diff = 0.30, wrist_roll = 0.0, depth_min = 0.40, depth_max = 1.20, width = 0.80, height_min = 0.40, height_max = 0.80, min_area = 250, max_area = 100000, plane = True, bigplane = True, vertical = False, tf_buffer = None, timeout = None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown', 'timeout'],
					   input_keys = ['fun_params', 'hsr', 'robot_info', 'start_time', 'stop_time'],
			    		   output_keys=['obj_name', 'obj_plane'])

		self.robot = robot

                #Start ObjectFinder client
                rospy.wait_for_service('/erasers/navigation/object_finder_srv')
                self.get_objects_centroid = rospy.ServiceProxy('/erasers/navigation/object_finder_srv', GetObjectsCentroid)

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.omni_base = self.robot.get("omni_base")
		self.gripper = self.robot.get('gripper')

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

		# yolo detection class
		self.detecion = MyDetection()

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

			self.whole_body.move_to_go()
			self.gripper.apply_force(1.0)
			
			if 1.0 - self.height_min < 0.40:
			    self.whole_body.move_to_joint_positions({'arm_lift_joint': 0.15})
			
			self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			rospy.sleep(self.delay)

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
			#yolo_tf detection
			out_img, color_img, depth_img, bb_list, obj_list = self.detecion.detection()
			rospy.logwarn("Objects detected: {}".format(obj_list))

			#rospy.loginfo(userdata.robot_info.obj_list)

			if (obj_list):
				# get minimum centroid comparing with yolo detection result
				centroid_center_x = objects.bbox[idx] + objects.bbox[2*n+idx]/2 # obj[i].x + obj[i].width/2
				centroid_center_y = objects.bbox[n+idx] + objects.bbox[3*n+idx]/2 # obj[i].y + obj[i].height/2
				centroid_center = np.array([centroid_center_x, centroid_center_y])

				prob = -1.0
				dist = 10000000 # initial value for comparison
				for i in range(len(bb_list)):
					_bbox = bb_list[i]

					_prob = -1.0
					for _obj in obj_list[i]:
						_prob = _obj[2]

					yolo_center_x = np.mean(_bbox[:2]) # mean[minX, maxX]
					yolo_center_y = np.mean(_bbox[2:]) # mean[minY, maxY]
					yolo_center = np.array([yolo_center_x, yolo_center_y])

					u = centroid_center - yolo_center
					_dist = np.linalg.norm(u)

					if _dist < dist and _prob > prob:
						dist = _dist
						prob = _prob
						idx_yolo = i

				rospy.logwarn("object: {}".format(obj_list[idx_yolo]))

				obj_name = None
				obj_prob = -1.0
				# compare yolo_BB and centroid.BB
				# TODO: exteralize config value (BB_DIST_THRESHOLD)
				BB_DIST_THRESHOLD = 300
				rospy.loginfo("The distance between yolo and pointcloud detection: {}, THRESHOLD: {} ".format(dist, BB_DIST_THRESHOLD)) 
				if dist > BB_DIST_THRESHOLD:
					rospy.loginfo("cannot recognize the object")
					obj_name = 'unknown'
				else:
					# chain detected label and ObjectList
					#rospy.logwarn(obj_list[idx_yolo])
					for _obj in obj_list[idx_yolo]:
						rospy.loginfo(_obj)
						if _obj[1].upper() in userdata.robot_info.obj_list.keys():
							obj_name = _obj[1]
							obj_prob = _obj[2]

				if obj_name is None:
					if len(obj_list) > 0:
						obj_name = 'unknown'
						obj_prob = -1.0
			else:
				obj_name = 'unknown'
				obj_prob = -1.0

			userdata.obj_name = obj_name.upper()
			rospy.logwarn("try to grasp {} {}".format(obj_name, obj_prob))

			# save image
			plt.imshow(out_img)
			now = strftime("%Y%m%d-%H%M%s", gmtime())
			plt.savefig(IMAGE_SAVE_PATH+"image_{}".format(now))
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

			self.whole_body.move_to_neutral()

			if (userdata.hsr == 46):
				drift_z = 0.02 #HSR46
			elif (userdata.hsr == 80):
				drift_z = 0.00 #HSR80
			else:
				drift_z = 0.00 #HSRSIM
			
			diff_z = 0.695 + arm_lift - (p[2] + drift_z)
			arm_flex = math.acos(1 - diff_z/0.345) #np.pi/3

			self.whole_body.move_to_joint_positions({'arm_flex_joint': -arm_flex,
							    'arm_lift_joint': arm_lift,#0.1382,
							    'arm_roll_joint': 0.,
							    'wrist_flex_joint': -np.pi/2+arm_flex,
							    'wrist_roll_joint': self.wrist_roll})

			self.gripper.command(0.9)

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

			#rospy.logwarn(diff_x)
			#rospy.logwarn(diff_y)

			linear_weight = self.whole_body.linear_weight
			angular_weight = self.whole_body.angular_weight

			self.whole_body.linear_weight = 1
			self.whole_body.angular_weight = 100

			#self.omni_base.go_rel(diff_x, diff_y, 0.0, 300.0)
			self.whole_body.move_end_effector_pose(((0, -1*diff_y, 0), (0, 0, 0, 1)), 'hand_palm_link')
			self.whole_body.move_end_effector_pose(((0, 0, diff_x), (0, 0, 0, 1)), 'hand_palm_link')

			self.whole_body.linear_weight = linear_weight
			self.whole_body.angular_weight = angular_weight

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

class DetectObjectPoseOnPlane(smach.State):
	def __init__(self, robot, delay = 1.0, tilt = -0.70, distance = 0.80, height_diff = 0.30, wrist_roll = 0.0, depth_min = 0.40, depth_max = 1.20, width = 0.80, height_min = 0.40, height_max = 0.80, min_area = 250, max_area = 100000, plane = True, bigplane = True, vertical = False, tf_buffer = None, timeout = None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown', 'timeout'],
					   input_keys = ['fun_params', 'hsr', 'robot_info', 'start_time', 'stop_time'],
			    		   output_keys=['obj_name', 'front_grasping'])

		self.robot = robot

                #Start ObjectFinder client
                rospy.wait_for_service('/erasers/navigation/object_finder_srv')
                self.get_objects_centroid = rospy.ServiceProxy('/erasers/navigation/object_finder_srv', GetObjectsCentroid)

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.omni_base = self.robot.get("omni_base")
		self.gripper = self.robot.get('gripper')

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

		# yolo detection class
		self.detecion = MyDetection()

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

			self.whole_body.move_to_go()
			self.gripper.apply_force(1.0)
			
			if 1.0 - self.height_min < 0.40:
			    self.whole_body.move_to_joint_positions({'arm_lift_joint': 0.15})
			
			self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			rospy.sleep(self.delay)

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
			#yolo_tf detection
			out_img, color_img, depth_img, bb_list, obj_list = self.detecion.detection()
			rospy.logwarn("Objects detected: {}".format(obj_list))

			#rospy.loginfo(userdata.robot_info.obj_list)

			if (obj_list):
				# get minimum centroid comparing with yolo detection result
				centroid_center_x = objects.bbox[idx] + objects.bbox[2*n+idx]/2 # obj[i].x + obj[i].width/2
				centroid_center_y = objects.bbox[n+idx] + objects.bbox[3*n+idx]/2 # obj[i].y + obj[i].height/2
				centroid_center = np.array([centroid_center_x, centroid_center_y])

				prob = -1.0
				dist = 10000000 # initial value for comparison
				for i in range(len(bb_list)):
					_bbox = bb_list[i]

					_prob = -1.0
					for _obj in obj_list[i]:
						_prob = _obj[2]

					yolo_center_x = np.mean(_bbox[:2]) # mean[minX, maxX]
					yolo_center_y = np.mean(_bbox[2:]) # mean[minY, maxY]
					yolo_center = np.array([yolo_center_x, yolo_center_y])

					u = centroid_center - yolo_center
					_dist = np.linalg.norm(u)

					if _dist < dist and _prob > prob:
						dist = _dist
						prob = _prob
						idx_yolo = i

				rospy.logwarn("object: {}".format(obj_list[idx_yolo]))

				obj_name = None
				obj_prob = -1.0
				# compare yolo_BB and centroid.BB
				# TODO: exteralize config value (BB_DIST_THRESHOLD)
				BB_DIST_THRESHOLD = 300
				rospy.loginfo("The distance between yolo and pointcloud detection: {}, THRESHOLD: {} ".format(dist, BB_DIST_THRESHOLD)) 
				if dist > BB_DIST_THRESHOLD:
					rospy.loginfo("cannot recognize the object")
					obj_name = 'unknown'
				else:
					# chain detected label and ObjectList
					#rospy.logwarn(obj_list[idx_yolo])
					for _obj in obj_list[idx_yolo]:
						rospy.loginfo(_obj)
						if _obj[1].upper() in userdata.robot_info.obj_list.keys():
							obj_name = _obj[1]
							obj_prob = _obj[2]

				if obj_name is None:
					if len(obj_list) > 0:
						obj_name = 'unknown'
						obj_prob = -1.0
			else:
				obj_name = 'unknown'
				obj_prob = -1.0

			userdata.obj_name = obj_name.upper()
			rospy.logwarn("try to grasp {} {}".format(obj_name, obj_prob))

			# save image
			plt.imshow(out_img)
			now = strftime("%Y%m%d-%H%M%s", gmtime())
			plt.savefig(IMAGE_SAVE_PATH+"image_{}".format(now))
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
			#userdata.obj_plane = p[2] - objects.plane_point[1]

			height_min = objects.plane_point[1] + 0.02
			height_max = objects.plane_point[1] + (self.height_max - self.height_min) - 0.02

			if (p[2] - self.height_min < 0.08):
				p[2] = self.height_min + 0.08

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

			self.whole_body.move_to_neutral()

			if (userdata.hsr == 46):
				drift_z = 0.02 #HSR46
			elif (userdata.hsr == 80):
				drift_z = 0.00 #HSR80
			else:
				drift_z = 0.00 #HSRSIM
		
			diff_z = 0.695 + arm_lift - (p[2] + drift_z)
			arm_flex = math.acos(1 - diff_z/0.345) #np.pi/3

			self.whole_body.move_to_joint_positions({'arm_flex_joint': -arm_flex,
							    'arm_lift_joint': arm_lift,#0.1382,
							    'arm_roll_joint': 0.,
							    'wrist_flex_joint': -np.pi/2+arm_flex,
							    'wrist_roll_joint': self.wrist_roll})

			self.gripper.command(0.9)

			if (userdata.hsr == 46):
				delta_dist = 0.00 #HSR46
			elif (userdata.hsr == 80):
				delta_dist = 0.02 #HSR80
			else:
				delta_dist = -0.02 #HSRSIM
		
			diff_x = p[0] + delta_dist - self.distance

			#TODO: Eigenvector alignment
			front_grasping = True
			if (abs(ev[0]) > abs(1.2*ev[2]) or abs(ev[1]) > abs(1.2*ev[2])):
				front_grasping = False

			userdata.front_grasping = front_grasping

			if (front_grasping):
				if (userdata.hsr == 46):
					diff_y = p[1] - 0.02 #HSR46
				elif (userdata.hsr == 80):
					diff_y = p[1] - 0.05 #HSR80
				else:
					diff_y = p[1] - 0.05 #HSRSIM

				linear_weight = self.whole_body.linear_weight
				angular_weight = self.whole_body.angular_weight

				self.whole_body.linear_weight = 1
				self.whole_body.angular_weight = 100

				#self.omni_base.go_rel(diff_x, diff_y, 0.0, 300.0)
				self.whole_body.move_end_effector_pose(((0, -1*diff_y, 0), (0, 0, 0, 1)), 'hand_palm_link')
				self.whole_body.move_end_effector_pose(((0, 0, diff_x), (0, 0, 0, 1)), 'hand_palm_link')

				self.whole_body.linear_weight = linear_weight
				self.whole_body.angular_weight = angular_weight

				###
				#From GraspVisual function
				if (userdata.hsr == 46):
					delta_dist = 0.00 #HSR46
				elif (userdata.hsr == 80):
					delta_dist = 0.00 #HSR80
				else:
					delta_dist = 0.00 #HSRSIM

				gripper2target_distance = self.distance - 0.28
				gripper2target_distance = gripper2target_distance - delta_dist

				arm_flex_angle =  self.whole_body.joint_positions['arm_flex_joint']
				diff_dist = gripper2target_distance - 0.345*math.sin( abs(arm_flex_angle) )

				self.whole_body.linear_weight = 1
				self.whole_body.angular_weight = 100

				self.whole_body.move_end_effector_pose(((0, 0, diff_dist), (0, 0, 0, 1)), 'hand_palm_link')

				self.whole_body.linear_weight = linear_weight
				self.whole_body.angular_weight = angular_weight

				self.gripper.apply_force(1.0)
				###
			else:
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
						diff_y = p[1] - 0.035 #HSRSIM

				linear_weight = self.whole_body.linear_weight
				angular_weight = self.whole_body.angular_weight

				self.whole_body.linear_weight = 1
				self.whole_body.angular_weight = 100

				#self.omni_base.go_rel(diff_x, diff_y, 0.0, 300.0)
				self.whole_body.move_end_effector_pose(((0, -1*diff_y, 0), (0, 0, 0, 1)), 'hand_palm_link')
				self.whole_body.move_end_effector_pose(((0, 0, diff_x), (0, 0, 0, 1)), 'hand_palm_link')

				self.whole_body.linear_weight = linear_weight
				self.whole_body.angular_weight = angular_weight

				#x positive to the front
				#y positive to the left
				#z positive upwards
				#theta = arccos ( <u,v> / (||u|| ||v||) )
				theta = np.arccos(ev[0])

				if (theta > 1.57):
					theta = theta - 3.1416

				if (abs(theta) > 1.57):
					theta = 1.57

				rospy.loginfo('closestobject.theta={}'.format(theta))

				###
				#From GraspDeltaVisual function
				delta_x = -0.19
				delta_y = 0.0#-0.08
				delta_z = 0.20

				arm_lift_height =  self.whole_body.joint_positions['arm_lift_joint']
				arm_flex_angle =  self.whole_body.joint_positions['arm_flex_joint']

				self.whole_body.move_to_joint_positions({'arm_lift_joint': arm_lift_height + 0.25})

				gripper2target_distance = self.distance - 0.28
				gripper2target_distance = gripper2target_distance - delta_x
				diff_dist = gripper2target_distance - 0.345*math.sin( abs(arm_flex_angle) )

				linear_weight = self.whole_body.linear_weight
				angular_weight = self.whole_body.angular_weight

				self.whole_body.linear_weight = 1
				self.whole_body.angular_weight = 100

				self.whole_body.move_end_effector_pose(((0, delta_y, 0), (0, 0, 0, 1)), 'hand_palm_link')

				self.whole_body.linear_weight = linear_weight
				self.whole_body.angular_weight = angular_weight

				rot_angle = np.pi+arm_flex_angle
				if (rot_angle > 1.92):
					rot_angle = 1.92
				self.whole_body.move_to_joint_positions({'wrist_flex_joint': -1*rot_angle})

				self.whole_body.move_end_effector_pose(((diff_dist, 0, 0), (0, 0, 0, 1)), 'hand_palm_link')

				self.whole_body.move_to_joint_positions({'wrist_roll_joint': theta})

				self.whole_body.move_to_joint_positions({'arm_lift_joint': arm_lift_height + delta_z})

				self.gripper.apply_force(1.0)
				###

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

class DetectObjectOnVerticalPlane(smach.State):
	def __init__(self, robot, delay = 1.0, tilt = -0.70, distance = 0.80, height_diff=0.30, wrist_roll = None, depth_min = 0.40, depth_max = 1.20, width = 0.80, height_min = 0.40, height_max = 0.80, min_area = 250, max_area = 100000, plane = True, bigplane = True, vertical = True, tf_buffer=None, timeout=None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown', 'timeout'],
					   input_keys = ['fun_params', 'hsr', 'robot_info', 'start_time', 'stop_time'],
			    		   output_keys=['obj_name'])

		self.robot = robot

                #Start ObjectFinder client
                rospy.wait_for_service('/erasers/navigation/object_finder_srv')
                self.get_objects_centroid = rospy.ServiceProxy('/erasers/navigation/object_finder_srv', GetObjectsCentroid)

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.omni_base = self.robot.get("omni_base")
		self.gripper = self.robot.get('gripper')

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

		# yolo detection class
		self.detecion = MyDetection()

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

			self.whole_body.move_to_go()
			self.gripper.apply_force(1.0)
			
			if 1.0 - self.height_min < 0.40:
			    self.whole_body.move_to_joint_positions({'arm_lift_joint': 0.15})
			
			self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			rospy.sleep(self.delay)

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
			#yolo_tf detection
			out_img, color_img, depth_img, bb_list, obj_list = self.detecion.detection()
			rospy.logwarn("Objects detected: {}".format(obj_list))

			if (obj_list):
				# get minimum centroid comparing with yolo detection result
				centroid_center_x = objects.bbox[idx] + objects.bbox[2*n+idx]/2 # obj[i].x + obj[i].width/2
				centroid_center_y = objects.bbox[n+idx] + objects.bbox[3*n+idx]/2 # obj[i].y + obj[i].height/2
				centroid_center = np.array([centroid_center_x, centroid_center_y])

				prob = -1.0
				dist = 10000000 # initial value for comparison
				for i in range(len(bb_list)):
					_bbox = bb_list[i]

					_prob = -1.0
					for _obj in obj_list[i]:
						_prob = _obj[2]

					yolo_center_x = np.mean(_bbox[:2]) # mean[minX, maxX]
					yolo_center_y = np.mean(_bbox[2:]) # mean[minY, maxY]
					yolo_center = np.array([yolo_center_x, yolo_center_y])

					u = centroid_center - yolo_center
					_dist = np.linalg.norm(u)

					if _dist < dist and _prob > prob:
						dist = _dist
						prob = _prob
						idx_yolo = i

				rospy.logwarn("object: {}".format(obj_list[idx_yolo]))

				obj_name = None
				obj_prob = -1.0
				# compare yolo_BB and centroid.BB
				# TODO: exteralize config value (BB_DIST_THRESHOLD)
				BB_DIST_THRESHOLD = 300
				rospy.loginfo("The distance between yolo and pointcloud detection: {}, THRESHOLD: {} ".format(dist, BB_DIST_THRESHOLD)) 
				if dist > BB_DIST_THRESHOLD:
					rospy.loginfo("cannot recognize the object")
					obj_name = 'unknown'
				else:
					# chain detected label and ObjectList
					#rospy.logwarn(obj_list[idx_yolo])
					for _obj in obj_list[idx_yolo]:
						rospy.loginfo(_obj)
						if _obj[1].upper() in userdata.robot_info.obj_list.keys():
							obj_name = _obj[1]
							obj_prob = _obj[2]

				if obj_name is None:
					if len(obj_list) > 0:
						obj_name = 'unknown'
						obj_prob = -1.0
			else:
				obj_name = 'unknown'
				obj_prob = -1.0

			userdata.obj_name = obj_name.upper()
			rospy.logwarn("try to grasp {} {}".format(obj_name, obj_prob))

			# save image
			plt.imshow(out_img)
			now = strftime("%Y%m%d-%H%M%s", gmtime())
			plt.savefig(IMAGE_SAVE_PATH+"image_{}".format(now))
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

			#height_min = objects.plane_point[1] + 0.02
			#height_max = objects.plane_point[1] + (self.height_max - self.height_min) - 0.02

			#if (p[2] - self.height_min < 0.05):
			#	p[2] = self.height_min + 0.05

			arm_lift = 0

			if (userdata.hsr == 46):
				drift_z = 0.04 #HSR46
			elif (userdata.hsr == 80):
				drift_z = 0.01 #HSR80
			else:
				drift_z = 0.05 #HSRSIM

			if 0.695 - (p[2] - drift_z) < 0.10:
				arm_lift = self.height_diff

			self.whole_body.move_to_neutral()

			diff_z = 0.695 + arm_lift - (p[2] + drift_z)
			arm_flex = math.acos(1 - diff_z/0.345) #np.pi/3

			self.whole_body.move_to_joint_positions({'arm_flex_joint': -arm_flex,
							    'arm_lift_joint': arm_lift,#0.1382,
							    'arm_roll_joint': 0.,
							    'wrist_flex_joint': -np.pi/2+arm_flex,
							    'wrist_roll_joint': 0.0})

			self.gripper.command(0.9)

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

			linear_weight = self.whole_body.linear_weight
			angular_weight = self.whole_body.angular_weight

			self.whole_body.linear_weight = 1
			self.whole_body.angular_weight = 100

			#self.omni_base.go_rel(diff_x, diff_y, 0.0, 300.0)
			self.whole_body.move_end_effector_pose(((0, -1*diff_y, 0), (0, 0, 0, 1)), 'hand_palm_link')
			self.whole_body.move_to_joint_positions({'wrist_roll_joint': self.wrist_roll})
			self.whole_body.move_end_effector_pose(((0, 0, diff_x), (0, 0, 0, 1)), 'hand_palm_link')

			self.whole_body.linear_weight = linear_weight
			self.whole_body.angular_weight = angular_weight

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

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

		self.robot = robot

                #Start ObjectFinder client
                rospy.wait_for_service('/erasers/navigation/object_finder_srv')
                self.get_objects_centroid = rospy.ServiceProxy('/erasers/navigation/object_finder_srv', GetObjectsCentroid)

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.omni_base = self.robot.get("omni_base")
		self.gripper = self.robot.get('gripper')

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

		# yolo detection class
		self.detecion = MyDetection()

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

			self.whole_body.move_to_go()
			self.gripper.apply_force(1.0)
			self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			rospy.sleep(self.delay)

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
			#yolo_tf detection
			out_img, color_img, depth_img, bb_list, obj_list = self.detecion.detection()
			rospy.logwarn("Objects detected: {}".format(obj_list))

			if (obj_list):
				# get minimum centroid comparing with yolo detection result
				centroid_center_x = objects.bbox[idx] + objects.bbox[2*n+idx]/2 # obj[i].x + obj[i].width/2
				centroid_center_y = objects.bbox[n+idx] + objects.bbox[3*n+idx]/2 # obj[i].y + obj[i].height/2
				centroid_center = np.array([centroid_center_x, centroid_center_y])

				prob = -1.0
				dist = 10000000 # initial value for comparison
				for i in range(len(bb_list)):
					_bbox = bb_list[i]

					_prob = -1.0
					for _obj in obj_list[i]:
						_prob = _obj[2]

					yolo_center_x = np.mean(_bbox[:2]) # mean[minX, maxX]
					yolo_center_y = np.mean(_bbox[2:]) # mean[minY, maxY]
					yolo_center = np.array([yolo_center_x, yolo_center_y])

					u = centroid_center - yolo_center
					_dist = np.linalg.norm(u)

					if _dist < dist and _prob > prob:
						dist = _dist
						prob = _prob
						idx_yolo = i

				rospy.logwarn("object: {}".format(obj_list[idx_yolo]))

				obj_name = None
				obj_prob = -1.0
				# compare yolo_BB and centroid.BB
				# TODO: exteralize config value (BB_DIST_THRESHOLD)
				BB_DIST_THRESHOLD = 300
				rospy.loginfo("The distance between yolo and pointcloud detection: {}, THRESHOLD: {} ".format(dist, BB_DIST_THRESHOLD)) 
				if dist > BB_DIST_THRESHOLD:
					rospy.loginfo("cannot recognize the object")
					obj_name = 'unknown'
				else:
					# chain detected label and ObjectList
					#rospy.logwarn(obj_list[idx_yolo])
					for _obj in obj_list[idx_yolo]:
						rospy.loginfo(_obj)
						if _obj[1].upper() in userdata.robot_info.obj_list.keys():
							obj_name = _obj[1]
							obj_prob = _obj[2]

				if obj_name is None:
					if len(obj_list) > 0:
						obj_name = 'unknown'
						obj_prob = -1.0
			else:
				obj_name = 'unknown'
				obj_prob = -1.0

			userdata.obj_name = obj_name.upper()
			rospy.logwarn("try to grasp {} {}".format(obj_name, obj_prob))

			# save image
			plt.imshow(out_img)
			now = strftime("%Y%m%d-%H%M%s", gmtime())
			plt.savefig(IMAGE_SAVE_PATH+"image_{}".format(now))
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

			#TODO: Eigenvector alignment
			front_grasping = True
			if (abs(ev[0]) > 1.2*abs(ev[2]) or abs(ev[1]) > 1.2*abs(ev[2])):
				front_grasping = False

			self.whole_body.move_to_neutral()
			self.gripper.command(1.2)

			if (front_grasping):
				if (userdata.hsr == 46):
					delta_dist = 0.04 #HSR46
				elif (userdata.hsr == 80):
					delta_dist = 0.02 #HSR80
				else:
					delta_dist = 0.13 #HSRSIM

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
						diff_y = p[1] - 0.0 #HSRSIM

				#rospy.logwarn(diff_x)
				#rospy.logwarn(diff_y)

				linear_weight = self.whole_body.linear_weight
				angular_weight = self.whole_body.angular_weight

				self.whole_body.linear_weight = 1
				self.whole_body.angular_weight = 100

				#self.omni_base.go_rel(diff_x, diff_y, 0.0, 300.0)
				self.whole_body.move_end_effector_pose(((0, -1*diff_y, 0), (0, 0, 0, 1)), 'hand_palm_link')

				arm_lift = 0
				if 0.695 - p[2] < 0.10:
					arm_lift = self.height_diff

				if (p[2] - self.height_min < 0.07):
					self.whole_body.move_to_neutral()
					self.whole_body.move_end_effector_pose(((0, 0, 0.60*diff_x), (0, 0, 0, 1)), 'hand_palm_link')

					#note: arm_flex_joint+wrist_flex_joint=1.57
					self.whole_body.move_to_joint_positions({'arm_flex_joint': -2.45,
								    'arm_lift_joint': 0.0,
								    'arm_roll_joint': 0.0,
								    'head_pan_joint': 0.0,
								    'head_tilt_joint': -0.32,
								    'wrist_flex_joint': 0.88,
								    'wrist_roll_joint': 0.0})

					self.whole_body.move_end_effector_pose(((0, 0, 0.40*diff_x), (0, 0, 0, 1)), 'hand_palm_link')

					#To grasp from the base, uncomment these lines
					#note: arm_flex_joint+wrist_flex_joint=1.57 
					self.whole_body.move_to_joint_positions({'arm_flex_joint': -2.45,
								    'arm_lift_joint': 0.0,
								    'arm_roll_joint': 0.0,
								    'head_pan_joint': 0.0,
								    'head_tilt_joint': -0.32,
								    'wrist_flex_joint': 0.75,
								    'wrist_roll_joint': 0.0})

				else:
					self.whole_body.move_to_neutral()
					self.whole_body.move_end_effector_pose(((0, 0, 0.20*diff_x), (0, 0, 0, 1)), 'hand_palm_link')

					#p[2] = self.height_min + 0.05
					if (userdata.hsr == 46):
						drift_z = 0.03 #HSR46
					elif (userdata.hsr == 80):
						drift_z = -0.01 #HSR80
					else:
						drift_z = -0.05 #HSRSIM

					diff_z = 0.695 + arm_lift - (p[2] + drift_z)
					arm_flex = math.acos(1 - diff_z/0.345) #np.pi/3

					self.whole_body.move_to_joint_positions({'arm_flex_joint': -arm_flex,
									    'arm_lift_joint': arm_lift,
									    'arm_roll_joint': 0.,
									    'wrist_flex_joint': -np.pi/2+arm_flex,
									    'wrist_roll_joint': 0.0})

					self.whole_body.move_end_effector_pose(((0, 0, 0.80*diff_x), (0, 0, 0, 1)), 'hand_palm_link')

				self.whole_body.linear_weight = linear_weight
				self.whole_body.angular_weight = angular_weight

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
					delta_dist = 0.0 #HSRSIM

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
						diff_y = p[1] - 0.035 #HSRSIM

				rospy.logwarn(diff_x)
				rospy.logwarn(diff_y)

				linear_weight = self.whole_body.linear_weight
				angular_weight = self.whole_body.angular_weight

				self.whole_body.linear_weight = 1
				self.whole_body.angular_weight = 100

				#self.omni_base.go_rel(diff_x, diff_y, 0.0, 300.0)
				self.whole_body.move_end_effector_pose(((0, -1*diff_y, 0), (0, 0, 0, 1)), 'hand_palm_link')
				self.whole_body.move_end_effector_pose(((0, 0, diff_x), (0, 0, 0, 1)), 'hand_palm_link')

				self.whole_body.linear_weight = linear_weight
				self.whole_body.angular_weight = angular_weight

				arm_lift = 0
				if 0.695 - p[2] < 0.10:
					arm_lift = self.height_diff

				if (p[2] - self.height_min < 0.05):
					p[2] = self.height_min + 0.05

				#note: arm_flex_joint+wrist_flex_joint=3.14
				#self.whole_body.move_to_joint_positions({'wrist_roll_joint': theta})
				self.whole_body.move_to_joint_positions({'arm_flex_joint': -1.57,
							    'arm_lift_joint': 0.0,
							    'arm_roll_joint': 0.0,
							    'head_pan_joint': 0.0,
							    'head_tilt_joint': -0.32,
							    'wrist_flex_joint': -1.57,
							    'wrist_roll_joint': theta})
				self.whole_body.move_to_joint_positions({'arm_flex_joint': -1.88,
							    'arm_lift_joint': 0.0,
							    'arm_roll_joint': 0.0,
							    'head_pan_joint': 0.0,
							    'head_tilt_joint': -0.32,
							    'wrist_flex_joint': -1.26,
							    'wrist_roll_joint': theta})

			self.gripper.apply_force(1.0)

			rospy.sleep(2)

			self.whole_body.move_to_go()

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

class DetectObjectOnFloor(smach.State):
	def __init__(self, robot, delay = 1.0, tilt = -0.96, height_diff = 0.30, wrist_roll = 0.0, depth_min = 0.20, depth_max = 1.20, width = 1.20, height_min = 0.0, height_max = 0.40, min_area = 250, max_area = 100000, plane = True, bigplane = False, vertical = False, tf_buffer=None, timeout=None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown', 'timeout'],
					   input_keys = ['fun_params', 'hsr', 'robot_info', 'start_time', 'stop_time'],
					   output_keys = ['objects', 'evectors', 'obj_names'])

		self.robot = robot

                #Start ObjectFinder client
                rospy.wait_for_service('/erasers/navigation/object_finder_srv')
                self.get_objects_centroid = rospy.ServiceProxy('/erasers/navigation/object_finder_srv', GetObjectsCentroid)

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.omni_base = self.robot.get("omni_base")
		self.gripper = self.robot.get('gripper')

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

		# yolo detection class
		self.detecion = MyDetection()

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

			self.whole_body.move_to_go()
			self.gripper.apply_force(1.0)
			self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			rospy.sleep(self.delay)

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
				_objects = self.get_objects_centroid(depth_min, depth_max, width_min, width_max, height_min, height_max, min_area, max_area, self.plane, self.bigplane, self.vertical).objects

				counter += 1
				if _objects.isobject == True or counter == 2:
					break

			if _objects.isobject == False:
				return 'unknown'

			#yolo_tf detection
			rospy.logwarn("Starting YOLO")
			out_img, color_img, depth_img, bb_list, obj_list = self.detecion.detection()
			rospy.logwarn("Stopping YOLO")

			#Get all onjects ordered by distance
			my_obj_names = []
			my_objects = []
			my_evectors = []
			my_idxs = []
			for j in range(_objects.n):
				#Get closest object index
				idx = -1
				n = _objects.n
				dist = 10000000
				for i in range(_objects.n):
					if not i in my_idxs:
						_dist = math.sqrt(math.pow(_objects.centroid[i], 2) + math.pow(_objects.centroid[2*n+i], 2))
						if (_dist < dist):
							dist = _dist
							idx = i 
				############
				############
				obj_name = None
				#rospy.loginfo(userdata.robot_info.obj_list)

				if (obj_list):
					# get minimum centroid comparing with yolo detection result
					centroid_center_x = _objects.bbox[idx] + _objects.bbox[2*n+idx]/2 # obj[i].x + obj[i].width/2
					centroid_center_y = _objects.bbox[n+idx] + _objects.bbox[3*n+idx]/2 # obj[i].y + obj[i].height/2
					centroid_center = np.array([centroid_center_x, centroid_center_y])

					prob = -1.0
					dist = 10000000 # initial value for comparison
					for i in range(len(bb_list)):
						_bbox = bb_list[i]

						_prob = -1.0
						for _obj in obj_list[i]:
							_prob = _obj[2]

						yolo_center_x = np.mean(_bbox[:2]) # mean[minX, maxX]
						yolo_center_y = np.mean(_bbox[2:]) # mean[minY, maxY]
						yolo_center = np.array([yolo_center_x, yolo_center_y])

						u = centroid_center - yolo_center
						_dist = np.linalg.norm(u)

						if _dist < dist and _prob > prob:
							dist = _dist
							prob = _prob
							idx_yolo = i

					rospy.logwarn("object: {}".format(obj_list[idx_yolo]))

					# compare yolo_BB and centroid.BB
					# TODO: exteralize config value (BB_DIST_THRESHOLD)
					BB_DIST_THRESHOLD = 25
					rospy.loginfo("The distance between yolo and pointcloud detection: {}, THRESHOLD: {} ".format(dist, BB_DIST_THRESHOLD)) 
					if dist > BB_DIST_THRESHOLD:
						rospy.logwarn("Cannot recognize the object")
						obj_name = 'unknown'
					else:
						# chain detected label and ObjectList
						rospy.loginfo(obj_list[idx_yolo])
						for _obj in obj_list[idx_yolo]:
							#rospy.loginfo(_obj)
							if _obj[1].upper() in userdata.robot_info.obj_list.keys():
								obj_name = _obj[1]

					if obj_name is None:
						if len(obj_list) > 0:
							obj_name = 'unknown'
				else:
					obj_name = 'unknown'

				# save image
				plt.imshow(out_img)
				now = strftime("%Y%m%d-%H%M%s", gmtime())
				plt.savefig(IMAGE_SAVE_PATH+"image_{}".format(now))
				############
				############

				#A point in robot coordinates is
				#x positive to the front
				#y positive to the left
				#z positive upwards
				p = [_objects.centroid[2*n+idx], -1*_objects.centroid[idx], _objects.centroid[n+idx] ]
				ev = [_objects.eigenvectors[2*3*n+3*idx], -1*_objects.eigenvectors[3*idx], _objects.eigenvectors[3*n+3*idx] ]
				#rospy.loginfo('object.base_footprint_point={}'.format(p))
				#rospy.loginfo('object.base_footprint_ev={}'.format(ev))

				my_obj_names.append(obj_name)
				my_objects.append(p)
				my_evectors.append(ev)
				my_idxs.append(idx)

			userdata.obj_names = my_obj_names
			userdata.objects = my_objects
			userdata.evectors = my_evectors

			rospy.loginfo('objects={}'.format(len(my_objects)))

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

class GraspObjectOnFloor(smach.State):
	def __init__(self, robot, height_diff = 0.30, wrist_roll = 0.0, height_min = 0.0, tf_buffer = None, timeout=None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown','timeout'],
					   input_keys = ['counter', 'objects', 'evectors', 'obj_names', 'hsr', 'start_time', 'stop_time'],
			    		   output_keys=['obj_name'])

		self.robot = robot

                #Start ObjectFinder client

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.omni_base = self.robot.get("omni_base")
		self.gripper = self.robot.get('gripper')

		self.wrist_roll = wrist_roll
		self.height_diff = height_diff
		self.height_min = height_min

	def execute(self, userdata):
		try:
			#A point in robot coordinates is
			#x positive to the front
			#y positive to the left
			#z positive upwards
			#rospy.loginfo('objects={}'.format(len(userdata.objects)))
			idx = userdata.counter - 1
			if idx >= len(userdata.objects):
				return 'unknown'

			userdata.obj_name = userdata.obj_names[idx].upper()
			rospy.logwarn("try to grasp {}".format(userdata.obj_names[idx]))

			p = [userdata.objects[idx][0], userdata.objects[idx][1], userdata.objects[idx][2] ]
			ev = [userdata.evectors[idx][0], userdata.evectors[idx][1], userdata.evectors[idx][2] ]
			rospy.loginfo('closestobject.base_footprint_point={}'.format(p))
			rospy.loginfo('closestobject.base_footprint_ev={}'.format(ev))

			#TODO: Eigenvector alignment
			front_grasping = True
			if (abs(ev[0]) > 1.2*abs(ev[2]) or abs(ev[1]) > 1.2*abs(ev[2])):
				front_grasping = False

			self.whole_body.move_to_neutral()
			self.gripper.command(1.2)

			if (front_grasping):
				if (userdata.hsr == 46):
					delta_dist = 0.04 #HSR46
				elif (userdata.hsr == 80):
					delta_dist = 0.02 #HSR80
				else:
					delta_dist = 0.02 #HSRSIM
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

				#rospy.logwarn(diff_x)
				#rospy.logwarn(diff_y)

				linear_weight = self.whole_body.linear_weight
				angular_weight = self.whole_body.angular_weight

				self.whole_body.linear_weight = 1
				self.whole_body.angular_weight = 100

				#self.omni_base.go_rel(diff_x, diff_y, 0.0, 300.0)
				self.whole_body.move_end_effector_pose(((0, -1*diff_y, 0), (0, 0, 0, 1)), 'hand_palm_link')

				arm_lift = 0
				if 0.695 - p[2] < 0.10:
					arm_lift = self.height_diff

				if (p[2] - self.height_min < 0.07):
					self.whole_body.move_to_neutral()
					self.whole_body.move_end_effector_pose(((0, 0, 0.70*diff_x), (0, 0, 0, 1)), 'hand_palm_link')

					#note: arm_flex_joint+wrist_flex_joint=1.57
					self.whole_body.move_to_joint_positions({'arm_flex_joint': -2.45,
								    'arm_lift_joint': 0.0,
								    'arm_roll_joint': 0.0,
								    'head_pan_joint': 0.0,
								    'head_tilt_joint': -0.32,
								    'wrist_flex_joint': 0.88,
								    'wrist_roll_joint': 0.0})

					self.whole_body.move_end_effector_pose(((0, 0, 0.30*diff_x), (0, 0, 0, 1)), 'hand_palm_link')

					#To grasp from the base, uncomment these lines
					#note: arm_flex_joint+wrist_flex_joint=1.57 (1.70)
					self.whole_body.move_to_joint_positions({'arm_flex_joint': -2.45,
								    'arm_lift_joint': 0.0,
								    'arm_roll_joint': 0.0,
								    'head_pan_joint': 0.0,
								    'head_tilt_joint': -0.32,
								    'wrist_flex_joint': 0.75,
								    'wrist_roll_joint': 0.0})

				else:
					self.whole_body.move_to_neutral()
					self.whole_body.move_end_effector_pose(((0, 0, 0.60*diff_x), (0, 0, 0, 1)), 'hand_palm_link')

					#p[2] = self.height_min + 0.05

					if (userdata.hsr == 46):
						drift_z = 0.03 #HSR46
					elif (userdata.hsr == 80):
						drift_z = -0.01 #HSR80
					else:
						drift_z = 0.03 #HSR46
					
					diff_z = 0.695 + arm_lift - (p[2] + drift_z)
					arm_flex = math.acos(1 - diff_z/0.345) #np.pi/3

					self.whole_body.move_to_joint_positions({'arm_flex_joint': -arm_flex,
									    'arm_lift_joint': arm_lift,
									    'arm_roll_joint': 0.,
									    'wrist_flex_joint': -np.pi/2+arm_flex,
									    'wrist_roll_joint': 0.0})

					self.whole_body.move_end_effector_pose(((0, 0, 0.40*diff_x), (0, 0, 0, 1)), 'hand_palm_link')

				self.whole_body.linear_weight = linear_weight
				self.whole_body.angular_weight = angular_weight

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
					delta_dist = -0.08 #HSR46
				elif (userdata.hsr == 80):
					delta_dist = -0.07 #HSR80
				else:
					delta_dist = -0.07 #HSRSIM

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
						diff_y = p[1] - 0.035 #HSRSIM

				rospy.logwarn(diff_x)
				rospy.logwarn(diff_y)

				linear_weight = self.whole_body.linear_weight
				angular_weight = self.whole_body.angular_weight

				self.whole_body.linear_weight = 1
				self.whole_body.angular_weight = 100

				#self.omni_base.go_rel(diff_x, diff_y, 0.0, 300.0)
				self.whole_body.move_end_effector_pose(((0, -1*diff_y, 0), (0, 0, 0, 1)), 'hand_palm_link')
				self.whole_body.move_end_effector_pose(((0, 0, diff_x), (0, 0, 0, 1)), 'hand_palm_link')

				self.whole_body.linear_weight = linear_weight
				self.whole_body.angular_weight = angular_weight

				arm_lift = 0
				if 0.695 - p[2] < 0.10:
					arm_lift = self.height_diff

				if (p[2] - self.height_min < 0.05):
					p[2] = self.height_min + 0.05

				#note: arm_flex_joint+wrist_flex_joint=3.14
				#self.whole_body.move_to_joint_positions({'wrist_roll_joint': theta})
				self.whole_body.move_to_joint_positions({'arm_flex_joint': -1.57,
							    'arm_lift_joint': 0.0,
							    'arm_roll_joint': 0.0,
							    'head_pan_joint': 0.0,
							    'head_tilt_joint': -0.32,
							    'wrist_flex_joint': -1.57,
							    'wrist_roll_joint': theta})
				self.whole_body.move_to_joint_positions({'arm_flex_joint': -1.84,
							    'arm_lift_joint': 0.0,
							    'arm_roll_joint': 0.0,
							    'head_pan_joint': 0.0,
							    'head_tilt_joint': -0.32,
							    'wrist_flex_joint': -1.30,
							    'wrist_roll_joint': theta})

			self.gripper.apply_force(1.0)

			self.whole_body.move_to_go()

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

class RecognizeObject(smach.State):
	def __init__(self, robot, point = (0.0, 0.0), p_dist = 150, tf_buffer = None, timeout = None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
					   input_keys = ['robot_info', 'start_time', 'stop_time'],
			    		   output_keys=['obj_name'])

		self.robot = robot

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.omni_base = self.robot.get("omni_base")
		self.gripper = self.robot.get('gripper')

		self.point = point
		self.p_dist = p_dist

		# yolo detection class
		self.detecion = MyDetection()

	def execute(self, userdata):
		try:
			rospy.sleep(1.0)
			############
			############
			#yolo_tf detection
			out_img, color_img, depth_img, bb_list, obj_list = self.detecion.detection()
			rospy.logwarn("Objects detected: {}".format(obj_list))

			#rospy.loginfo(userdata.robot_info.obj_list)

			if (obj_list):
				# get minimum centroid comparing with yolo detection result
				centroid_center = np.array([self.point[0], self.point[1]])

				idx_yolo = -1
				prob = -1.0
				dist_max = self.p_dist # initial value for comparison
				dist = 100000
				for i in range(len(bb_list)):
					_bbox = bb_list[i]

					_prob = -1.0
					for _obj in obj_list[i]:
						_prob = _obj[2]

					yolo_center_x = np.mean(_bbox[:2]) # mean[minX, maxX]
					yolo_center_y = np.mean(_bbox[2:]) # mean[minY, maxY]
					yolo_center = np.array([yolo_center_x, yolo_center_y])

					u = centroid_center - yolo_center
					_dist = np.linalg.norm(u)

					if _dist < dist_max and _prob > prob:
						#rospy.logwarn("yolo_center: {}".format(yolo_center))
						dist = _dist
						prob = _prob
						idx_yolo = i

				if (idx_yolo > -1):
					rospy.logwarn("object: {}".format(obj_list[idx_yolo]))

					obj_name = None
					obj_prob = -1.0
					# compare yolo_BB and centroid.BB
					# TODO: exteralize config value (BB_DIST_THRESHOLD)
					BB_DIST_THRESHOLD = self.p_dist
					rospy.loginfo("The distance between yolo and pointcloud detection: {}, THRESHOLD: {} ".format(dist, BB_DIST_THRESHOLD)) 
					if dist > BB_DIST_THRESHOLD:
						rospy.loginfo("cannot recognize the object")
						obj_name = 'unknown'
					else:
						# chain detected label and ObjectList
						#rospy.logwarn(obj_list[idx_yolo])
						for _obj in obj_list[idx_yolo]:
							rospy.loginfo(_obj)
							if _obj[1].upper() in userdata.robot_info.obj_list.keys():
								obj_name = _obj[1]
								obj_prob = _obj[2]

					if obj_name is None:
						if len(obj_list) > 0:
							obj_name = 'unknown'
							obj_prob = -1.0
				else:
					obj_name = 'unknown'
					obj_prob = -1.0
			else:
				obj_name = 'unknown'
				obj_prob = -1.0

			userdata.obj_name = obj_name.upper()
			rospy.logwarn("try to grasp {} {}".format(obj_name, obj_prob))

			# save image
			plt.imshow(out_img)
			now = strftime("%Y%m%d-%H%M%s", gmtime())
			plt.savefig(IMAGE_SAVE_PATH+"image_{}".format(now))
			############
			############

			self.whole_body.move_to_go()

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

class RecognizeObjectOnHand(smach.State):
	def __init__(self, robot, point = (0.0, 0.0), p_dist = 150, tf_buffer = None, timeout = None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
					   input_keys = ['robot_info', 'start_time', 'stop_time'],
			    		   output_keys=['obj_name'])

		self.robot = robot

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.omni_base = self.robot.get("omni_base")
		self.gripper = self.robot.get('gripper')

		self.point = point
		self.p_dist = p_dist

		# yolo detection class
		self.detecion = MyDetection()

	def execute(self, userdata):
		try:
			#self.whole_body.move_to_joint_positions({'arm_flex_joint': -1.20,
			#			    'arm_lift_joint': 0.0,
			#			    'arm_roll_joint': 3.10,
			#			    'head_pan_joint': 0.0,
			#			    'head_tilt_joint': -0.90,
			#			    'wrist_flex_joint': -1.57,
			#			    'wrist_roll_joint': 0.0})

			self.whole_body.move_to_joint_positions({'arm_flex_joint': -0.85,
						                 'arm_lift_joint': 0.0,
						                 'arm_roll_joint': 3.40,
						                 'head_pan_joint': 0.0,
						                 'head_tilt_joint': -0.92,
						                 'wrist_flex_joint': -1.519,
						                 'wrist_roll_joint': 0.3})

			rospy.sleep(1.0)
			############
			############
			#yolo_tf detection
			out_img, color_img, depth_img, bb_list, obj_list = self.detecion.detection()
			rospy.logwarn("Objects detected: {}".format(obj_list))

			#rospy.loginfo(userdata.robot_info.obj_list)

			if (obj_list):
				# get minimum centroid comparing with yolo detection result
				centroid_center = np.array([self.point[0], self.point[1]])

				idx_yolo = -1
				prob = -1.0
				dist_max = self.p_dist # initial value for comparison
				dist = 10000000
				for i in range(len(bb_list)):
					_bbox = bb_list[i]

					_prob = -1.0
					for _obj in obj_list[i]:
						_prob = _obj[2]

					yolo_center_x = np.mean(_bbox[:2]) # mean[minX, maxX]
					yolo_center_y = np.mean(_bbox[2:]) # mean[minY, maxY]
					yolo_center = np.array([yolo_center_x, yolo_center_y])

					u = centroid_center - yolo_center
					_dist = np.linalg.norm(u)

					if _dist < dist_max and _prob > prob:
						dist = _dist
						prob = _prob
						idx_yolo = i

				if (idx_yolo > -1):
					rospy.logwarn("Object in list: {}".format(obj_list[idx_yolo]))

					obj_name = None
					obj_prob = -1.0
					# compare yolo_BB and centroid.BB
					# TODO: exteralize config value (BB_DIST_THRESHOLD)
					BB_DIST_THRESHOLD = self.p_dist
					rospy.loginfo("The distance between yolo and pointcloud detection: {}, THRESHOLD: {} ".format(dist, BB_DIST_THRESHOLD)) 
					if dist > BB_DIST_THRESHOLD:
						rospy.loginfo("cannot recognize the object")
						obj_name = 'unknown'
					else:
						# chain detected label and ObjectList
						#rospy.logwarn(obj_list[idx_yolo])
						for _obj in obj_list[idx_yolo]:
							rospy.loginfo(_obj)
							if _obj[1].upper() in userdata.robot_info.obj_list.keys():
								obj_name = _obj[1]
								obj_prob = _obj[2]

					if obj_name is None:
						if len(obj_list) > 0:
							obj_name = 'unknown'
							obj_prob = -1.0
				else:
					obj_name = 'unknown'
					obj_prob = -1.0
			else:
				obj_name = 'unknown'
				obj_prob = -1.0

			userdata.obj_name = obj_name.upper()
			rospy.logwarn("Object in hand {} {}".format(obj_name, obj_prob))

			# save image
			plt.imshow(out_img)
			now = strftime("%Y%m%d-%H%M%s", gmtime())
			plt.savefig(IMAGE_SAVE_PATH+"image_{}".format(now))
			############
			############

			self.whole_body.move_to_go()

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'

class DetectSpaceOnPlane(smach.State):
	def __init__(self, robot, delay = 1.0, tilt = -0.70, distance = 0.80, height_diff = 0.30, wrist_roll = 0.0, depth_min = 0.40, depth_max = 1.20, width = 0.80, height_min = 0.40, height_max = 0.80, dist2plane = 0.08, wside = 0.10, dside = 0.10, delta_lift = 0.10, h_delta = 0.15, side = 0, tf_buffer = None, timeout = None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'unknown', 'timeout'],
					   input_keys = ['fun_params', 'hsr', 'start_time', 'stop_time'])

		self.robot = robot

                #Start SpaceFinder client
                rospy.wait_for_service('/erasers/navigation/space_finder_srv')
                self.get_space_centroid = rospy.ServiceProxy('/erasers/navigation/space_finder_srv', GetSpaceCentroid)

		if tf_buffer:
			self.tf_buffer = tf_buffer
		else:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
			tf2_ros.TransformListener(self.tf_buffer)

		#self.xtion = HSRB_Xtion(tf_buffer=self.tf_buffer)
		self.whole_body = self.robot.get('whole_body')
		self.omni_base = self.robot.get("omni_base")
		self.gripper = self.robot.get('gripper')

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

		self.dist2plane = dist2plane
		self.wside = wside
		self.dside = dside
		self.delta_lift = delta_lift
		self.side = side
		self.h_delta = h_delta

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

			space_info = userdata.fun_params.space_info
			if (space_info.in_use):
				if (space_info.dist2plane):
					self.dist2plane = space_info.dist2plane

				if (space_info.wside):
					self.wside = space_info.wside

				if (space_info.dside):
					self.dside = space_info.dside

				if (space_info.delta_lift):
					self.delta_lift = space_info.delta_lift

				if (space_info.side):
					self.side = space_info.side

			self.whole_body.move_to_go()

			if 1.0 - self.height_min < 0.40:
			    self.whole_body.move_to_joint_positions({'arm_lift_joint': 0.25})

			self.whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': self.tilt})
			rospy.sleep(self.delay)

			depth_min = self.depth_min
			depth_max = self.depth_max
			width_min = -1*self.width/2
			width_max = self.width/2
			height_min = self.height_min
			height_max = self.height_max

			distance2plane = self.dist2plane

			wside = self.wside
			dside = self.dside

			##################
                        #Find closest object centroid
			counter = 0
			while True:
				#Call service with parameter (depth_min, depth_max, width_min, width_max, height_min, height_max, side_width, side_depth)
				#Output in camera coordinates WRT the robot
				#i_width positive to the right
				#j_height positive upwards
				#k_depth positive to the front
				#centroid = [i_width, j_height, k_depth]
				space = self.get_space_centroid(depth_min, depth_max, width_min, width_max, height_min, height_max, self.wside, self.dside).space

				counter += 1
				if space.isspace == True or counter == 2:
					break

			if space.isspace == False:
				return 'unknown'

			#A point in robot coordinates is
			#x positive to the front
			#y positive to the left
			#z positive upwards
			if (self.side == 0):
				p = [space.backleft[2], -1*space.backleft[0], space.backleft[1] ]
				rospy.loginfo('freespace.base_footprint_backleft={}'.format(p))
				if (userdata.hsr == 46):
					drift_y = -0.06 #HSR46
				elif (userdata.hsr == 80):
					drift_y = -0.06 #HSR80
				else:
					drift_y = -0.06 #HSRSIM
	
			elif (self.side == 1):
				p = [space.backright[2], -1*space.backright[0], space.backright[1] ]
				rospy.loginfo('freespace.base_footprint_backright={}'.format(p))
				if (userdata.hsr == 46):
					drift_y = 0.06 #HSR46
				elif (userdata.hsr == 80):
					drift_y = 0.06 #HSR80
				else:
					drift_y = 0.06 #HSR46

			elif (self.side == 2):
				p = [space.frontleft[2], -1*space.frontleft[0], space.frontleft[1] ]
				rospy.loginfo('freespace.base_footprint_frontleft={}'.format(p))
				if (userdata.hsr == 46):
					drift_y = -0.08 #HSR46
				elif (userdata.hsr == 80):
					drift_y = -0.08 #HSR80
				else:
					drift_y = -0.08 #HSRSIM

			elif (self.side == 3):
				p = [space.frontright[2], -1*space.frontright[0], space.frontright[1] ]
				rospy.loginfo('freespace.base_footprint_frontright={}'.format(p))
				if (userdata.hsr == 46):
					drift_y = 0.06 #HSR46
				elif (userdata.hsr == 80):
					drift_y = 0.06 #HSR80
				else:
					drift_y = 0.06 #HSRSIM
			elif (self.side == 4):
				p_1 = [space.backleft[2], -1*space.backleft[0], space.backleft[1] ]
				p_2 = [space.frontleft[2], -1*space.frontleft[0], space.frontleft[1] ]

				rospy.loginfo('freespace.left_difference={}'.format(abs(p_2[1]-p_1[1])))
				if (abs(p_2[1]-p_1[1]) >= 0.75*wside):
					p = p_2
					rospy.loginfo('freespace.base_footprint_frontleft={}'.format(p))
					if (userdata.hsr == 46):
						drift_y = -0.08 #HSR46
					elif (userdata.hsr == 80):
						drift_y = -0.08 #HSR80
					else:
						drift_y = -0.08 #HSR46
				else:
					p = p_1
					rospy.loginfo('freespace.base_footprint_backleft={}'.format(p))
					if (userdata.hsr == 46):
						drift_y = -0.06 #HSR46
					elif (userdata.hsr == 80):
						drift_y = -0.06 #HSR80
					else:
						drift_y = -0.06 #HSRSIM

			elif (self.side == 5):
				p_1 = [space.backright[2], -1*space.backright[0], space.backright[1] ]
				p_2 = [space.frontright[2], -1*space.frontright[0], space.frontright[1] ]

				rospy.loginfo('freespace.right_difference={}'.format(abs(p_2[1]-p_1[1])))
				if (abs(p_2[1]-p_1[1]) >= 0.75*wside):
					p = p_2
					rospy.loginfo('freespace.base_footprint_frontright={}'.format(p))
					if (userdata.hsr == 46):
						drift_y = 0.06 #HSR46
					elif (userdata.hsr == 80):
						drift_y = 0.00 #HSR80
					else:
						drift_y = 0.00 #HSRSIM
				else:
					p = p_1
					rospy.loginfo('freespace.base_footprint_backright={}'.format(p))
					if (userdata.hsr == 46):
						drift_y = 0.06 #HSR46
					elif (userdata.hsr == 80):
						drift_y = 0.06 #HSR80
					else:
						drift_y = 0.06 #HSRSIM

			else:
				p = [space.backleft[2], -1*space.backleft[0], space.backleft[1] ]
				rospy.loginfo('freespace.base_footprint_backleft={}'.format(p))
				if (userdata.hsr == 46):
					drift_y = -0.06 #HSR46
				elif (userdata.hsr == 80):
					drift_y = -0.06 #HSR80
				else:
					drift_y = -0.06 #HSRSIM

			#0.05 m is the minimum h_delta
			if self.h_delta < 0.05:
				self.h_delta = 0.05
			p[2] = p[2] + self.h_delta

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

			self.whole_body.move_to_neutral()

			if (userdata.hsr == 46):
				drift_z = 0.02 #HSR46
			elif (userdata.hsr == 80):
				drift_z = 0.02 #HSR80
			else:
				drift_z = 0.02 #HSRSIM

			diff_z = 0.695 + arm_lift - (p[2] + drift_z)
			arm_flex = math.acos(1 - diff_z/0.345) #np.pi/3

			self.whole_body.move_to_joint_positions({'arm_flex_joint': -arm_flex,
							    'arm_lift_joint': arm_lift,#0.1382,
							    'arm_roll_joint': 0.,
							    'wrist_flex_joint': -np.pi/2+arm_flex,
							    'wrist_roll_joint': self.wrist_roll})

			#self.gripper.command(0.9)
			diff_x = p[0] - self.distance
			diff_y = p[1] + drift_y

			rospy.logwarn(diff_x)
			rospy.logwarn(diff_y)

			if (userdata.hsr == 46):
				target = 0.08 #HSR46
			elif (userdata.hsr == 80):
				target = 0.08 #HSR80
			else:
				target = 0.08 #HSRSIM

			gripper2target_distance = self.distance - 0.28
			gripper2target_distance = gripper2target_distance - target
			diff_dist = gripper2target_distance - 0.345*math.sin( abs(arm_flex) )

			linear_weight = self.whole_body.linear_weight
			angular_weight = self.whole_body.angular_weight

			self.whole_body.linear_weight = 1
			self.whole_body.angular_weight = 100

			#self.omni_base.go_rel(diff_x, diff_y, 0.0, 300.0)
			self.whole_body.move_end_effector_pose(((0, -1*diff_y, 0), (0, 0, 0, 1)), 'hand_palm_link')
			self.whole_body.move_end_effector_pose(((0, 0, diff_x+diff_dist), (0, 0, 0, 1)), 'hand_palm_link')

			#Minimun hand distance to plane is 0.05 m
			diff_z = 0.695 + arm_lift - (p[2] + drift_z - (self.h_delta - distance2plane))
			arm_flex = math.acos(1 - diff_z/0.345) #np.pi/3
			self.whole_body.move_to_joint_positions({'arm_flex_joint': -arm_flex,
							    'arm_lift_joint': arm_lift,#0.1382,
							    'arm_roll_joint': 0.,
							    'wrist_flex_joint': -np.pi/2+arm_flex,
							    'wrist_roll_joint': self.wrist_roll})

			self.gripper.command(0.9)

			diff_z = 0.695 + arm_lift - (p[2] + drift_z)
			arm_flex = math.acos(1 - diff_z/0.345) #np.pi/3
			self.whole_body.move_to_joint_positions({'arm_flex_joint': -arm_flex,
							    'arm_lift_joint': arm_lift + self.delta_lift,#0.1382,
							    'arm_roll_joint': 0.,
							    'wrist_flex_joint': -np.pi/2+arm_flex,
							    'wrist_roll_joint': self.wrist_roll})

			self.whole_body.move_end_effector_pose(((0, 0, -1*(diff_x+diff_dist)), (0, 0, 0, 1)), 'hand_palm_link')

			self.whole_body.linear_weight = linear_weight
			self.whole_body.angular_weight = angular_weight

			self.whole_body.move_to_neutral()

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

			#return 'timeout'
		except:
			rospy.logerr(traceback.format_exc())
			return 'failure'
