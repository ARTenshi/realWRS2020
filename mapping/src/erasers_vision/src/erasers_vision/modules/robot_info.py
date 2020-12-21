#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Handling HSR information.
"""
from IPython.core.debugger import Tracer; keyboard = Tracer()
import rospy
import rospkg
from xml.etree import ElementTree
import yaml
import tf
import time
import sys
import os
sys.path.append(
    "/".join(os.path.abspath(os.path.dirname(".")).split("/")[:-1]))

class Object:
	name = ''
	category = ''
	defaultLocation = ''
	room = ''

class Position:
    x = 0.
    y = 0.
    z = 0.
    yaw = 0.

class Location:
    position = Position()
    put_position = Position()
    name = ''
    room = ''
    isDoor = ''

class RobotInfo(object):
    """
    Handling HSR information
    """

    def __init__(self):

        # make obj and locaiton list
        self.obj_list = {}
        self.location_list = {}
        self.obj_list_path = None
        self.obsrv_pt_list = []

    def make_obj_list(self, task):
        """
        make_obj_list
        Parameters
        ----
        room : room name to tidy up

        Returns
        ---
        dictionary of object list
        """
        PATH_TO_XML = "/home/roboworks/catkin_ws/src/"

        # which room
        if task == "tidy up":
            PATH_TO_XML += "jpop2019/data/objects_tidy_up.xml"
        elif task == "go get it":
            PATH_TO_XML += "jpop2019/data/objects_go_get_it.xml"
        else:
            PATH_TO_XML += "jpop2019/data/objects.xml"
        self.obj_list_path = PATH_TO_XML

        rospy.loginfo("parsing {}".format(PATH_TO_XML))
        result = {}
        tree = ElementTree.parse(PATH_TO_XML)
        for category in tree.getroot():
            for elem in category:
                targetObject = Object()
                targetObject.name = elem.attrib['name']
                targetObject.category = category.attrib['name']
                targetObject.room = category.attrib['room']
                targetObject.defaultLocation = elem.attrib['defaultLocation']
                result[targetObject.name.upper()] = targetObject
        self.obj_list = result
    
    def make_location_list(self, task):
        """
        make_location_list

        Returns
        ---
        dictionary of object list
        """

        PATH_TO_XML = "/home/roboworks/catkin_ws/src/"
        # which room
        if task == "tidy up":
            PATH_TO_XML += "jpop2019/data/locations_tidy_up.xml"
            self.start_area = "living room"
        elif task == "go get it":
            PATH_TO_XML += "jpop2019/data/locations_go_get_it.xml"
            self.start_area = "living room"
        else:
            PATH_TO_XML += "jpop2019/data/locations.xml"
            self.start_area = "living room"

        self.locate_list_path = PATH_TO_XML
        print "parsing {}".format(PATH_TO_XML)
    
        result = {}
        room_list = {}
        tree = ElementTree.parse(PATH_TO_XML)
        for room in tree.getroot():
            # make room_list
            if room.attrib["name"] not in room_list.keys():
                position = Position()
                str_position = room.attrib['position']
                position.x = float(str_position.split(" ")[0])
                position.y = float(str_position.split(" ")[1])
                position.yaw = float(str_position.split(" ")[3])
                room_list[room.attrib["name"]]  = position
            # make poition_list
            for elem in room:
                targetLocation = Location()
                targetPosition = Position()
                putPosition = Position()
                targetLocation.name = elem.attrib['name']
                targetLocation.room = room.attrib['name']
                targetLocation.isDoor = elem.attrib['isDoor']
                str_position = elem.attrib['global_position']
                targetPosition.x = float(str_position.split(" ")[0])
                targetPosition.y = float(str_position.split(" ")[1])
                targetPosition.z = float(str_position.split(" ")[2])
                targetPosition.yaw = float(str_position.split(" ")[3])
                targetLocation.position = targetPosition
                str_position = elem.attrib['put_position']
                putPosition.x = float(str_position.split(" ")[0])
                putPosition.y = float(str_position.split(" ")[1])
                putPosition.z = float(str_position.split(" ")[2])
                targetLocation.put_position = putPosition
                result[targetLocation.name.upper()] = targetLocation
        
        self.location_list = result
        self.room_list = room_list
        # make observe point list
        self._make_obsrv_pt_list()

    def _make_obsrv_pt_list(self):
        """
        make obsrv point list from location list made by itself
        """
        obsrv_pt_list = []
        for k, v in self.location_list.iteritems():
            if "SEARCH" in k:
                obsrv_pt_list.append(v)

        for i in range(len(obsrv_pt_list)):
            for j in range(len(obsrv_pt_list)):
                if (str(i)+' ') in obsrv_pt_list[j].name:
                    self.obsrv_pt_list.append(obsrv_pt_list[j])

    def get_obj_list(self):
        return self.obj_list

    def save_info(self, save_path):
        """
        save current info
        """
        # save which obj list is used
        pkg = rospkg.rospack.RosPack().get_path('wrs2020')
        f = open(pkg+"/check_points/"+save_path, "w")
        
        f.write(yaml.dump({"obj_list_path": self.obj_list_path}, default_flow_style=False))    
        f.write(yaml.dump({"start_time": self.start_time}, default_flow_style=False))    
        f.write(yaml.dump({"start_area": self.start_area}, default_flow_style=False))
        
        # save abs position
        pos = self.omni_base.get_pose()
        yaw = tf.transformations.euler_from_quaternion(pos.ori)[2]
        f.write(yaml.dump({"pos_x": pos.pos.x, "pos_y": pos.pos.y, "pos_yaw": yaw}, default_flow_style=False))

        f.close()

    def restore_info(self, save_path):
        """
        restore info from saved yaml file
        """
        pkg = rospkg.rospack.RosPack().get_path('wrs2020')
        f = open(pkg+"/check_points/"+save_path, "r")
        info = yaml.load(f)
        f.close()
        
        # restore start area
        self.start_area = info["start_area"]
        self.start_time = info["start_time"]

        # restore obj list and location list
        if "Objects1" in info["obj_list_path"]:
            self.make_obj_list("children room")
            self.make_location_list("children room")
            rospy.loginfo("restore object1")
        elif "Objects2" in info["obj_list_path"]:
            self.make_obj_list("living and dining room")
            self.make_location_list("living and dining room")
            rospy.loginfo("restore object2")
        else:
            rospy.logerr("failed to restore robot info")
            return 0
        
      
