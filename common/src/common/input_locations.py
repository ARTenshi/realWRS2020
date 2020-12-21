#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import xml.etree.ElementTree as ET
from common.grammar import LocationInfo
import hsrb_interface
import re

#rospy.init_node('input_locations')

robot = hsrb_interface.Robot()
whole_body = robot.get('whole_body')
omni_base = robot.get('omni_base')
print 'create hsrb_interface'

rospy.Rate(20)

class InputLocations():
    def file_editing(self):
        with open("location_dict.py", "r") as f:
            self.read_file = f.readlines()
            #print 'readlines:',self.read_file
        self.read_file = "".join(self.read_file)
        #print 'after join:',self.read_file
        matchObj = re.search('LOCATIONS\s*=\s*\{([^\}]*\n)*[^\}]*\}',self.read_file)
        if matchObj is not None:
            match_file = matchObj.span()
        else:
            match_file = (-1,-1)
        #print 'match_file:',match_file
        span_file = self.read_file[match_file[0]:match_file[1]]
        #print 'span_file:',span_file

        # give location from xml
        xml_locations = list(LocationInfo('/home/hsr-user/GPSRCmdGen/GPSRCmdGen/Resources/Hokuto.xml').locations)

        x = .0
        y = .0
        w = .0
        Coordinate = ((x,y),w)
        rate = rospy.Rate(10.0)
        #rate = rospy.Rate(.001)
        # make dict
        location_dict={}

        if matchObj is not None:
            location_dict = eval(span_file.strip('LOCATINS').strip().strip('=').strip())

        for key in xml_locations:
            if key not in location_dict:
                location_dict[key] = Coordinate
        print 'first_position:',location_dict
        while not rospy.is_shutdown():
            print 'input location name:'
            ri = raw_input()
            #ri = 'teepee'
            if ri in location_dict:
                print 'success'
                position = omni_base.pose
                #position = (0.111111111,0.211111111,0.3111111111)
                x,y,w = round(position[0],3),round(position[1],3),round(position[2],3)
                Coordinate = ((x,y),w)
                print 'Coordinate:',Coordinate
                location_dict.update({ri:(Coordinate)})
            else:
                print 'failure'
            print 'location_dict:',location_dict
            #print '\n'
            rate.sleep()
            with open("location_dict.py", "w") as f:
                f.write(self.read_file[:match_file[0]])
                if matchObj is None:
                    f.write('\n')
                    f.write('\n')
                f.write('LOCATIONS = ')
                f.write(str(location_dict))
                f.write(self.read_file[match_file[1]:])

input_locations = InputLocations()
input_locations.file_editing()
