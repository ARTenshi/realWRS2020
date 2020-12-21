#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Handling Function Parameters.
"""

import rospy
import rospkg

#Function parameter classes
class RobotPose:
	delay = []
	tilt = []
	distance = []
	height_diff = []
	wrist_roll = []
	in_use = False

class EdgesBoundingBox:
	position = []
	orientation = []
	maxx = []
	miny = []
	maxy = []
	maxz = []
	in_use = False

class PointsBoundingBox:
	maxx = []
	miny = []
	maxy = []
	maxz = []
	in_use = False

class ObjectsBoundingBox:
	depth_min = []
	depth_max = []
	depth = []
	width = []
	height_min = []
	height_max = []
	in_use = False

class PlaneInfo:
	min_area = []
	max_area = []
	plane = []
	bigplane = []
	vertical = []
	in_use = False

class SpaceInfo:
	dist2plane = []
	wside = []
	dside = []
	delta_lift = []
	side = []
	in_use = False

class FunctionParams:
	robot_pose = RobotPose()
	edges_bb = EdgesBoundingBox()
	points_bb = PointsBoundingBox()
	objects_bb = ObjectsBoundingBox()
	plane_info = PlaneInfo()
	space_info = SpaceInfo()

