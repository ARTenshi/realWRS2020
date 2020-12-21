#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import datetime
import cv2
import numpy as np
import time
import smach
import smach_ros
from common import publish_initial_pose
from common.smach_states import *
import os
import traceback
from pocketsphinx_jsgf import PocketSphinxClient
from common import speech
from common import grammar
import re
import hsrb_interface
robot = hsrb_interface.Robot()
sphinx = PocketSphinxClient()

#INFO_DIR = '/home/hsr-user/Documents/GPSRCmdGen/GPSRCmdGen/Resources'
INFO_DIR = '/home/hsr-user/Documents/GPSRCmdGen/GPSRCmdGen/Resources'
LOC_INFO = grammar.LocationInfo(os.path.join(INFO_DIR, 'Locations.xml'))
import common.roboworks as ARENA

GRAMMAR = """
#JSGF V1.0;
grammar commands;
public <navigate> = go to the <location>;
"""
GRAMMAR += '<location> = ' + '|'.join(LOC_INFO.locations)+';\n'
SAY = speech.DefaultTTS().say
print GRAMMAR
def parse_command(cmd):
    rospy.loginfo(cmd)
    if re.search('(^|\s)go(\s|$)', cmd):
        return 'NAVIGATE'
    #if re.search('(^|\s)(\s|$)'):

def add_subsms():
    @smach.cb_interface(outcomes=['failure','NAVIGATE'],
                        input_keys=['command'],
                        output_keys=['params'])
    def parse_cb(userdata):
        cmd = userdata.command
        cmd_type = parse_command(cmd)
        params = {}
        userdata.params = params
        if cmd_type == 'NAVIGATE':
            m = re.search('(^|\s)(?P<loc>'+'|'.join(LOC_INFO.locations)+')(\s|$)', cmd)
            if not m or not m.groupdict()['loc']:
                SAY('Where shall I go?')
            params['pose'] = ARENA.LOCATIONS[m.groupdict()['loc']]
        return cmd_type
    smach.StateMachine.add('PARSE_COMMAND', smach.CBState(parse_cb),
                           transitions = {'failure': 'WAIT_HAND', 'NAVIGATE':'NAVIGATE'},
                           remapping = {'params': 'params'})
    smach.StateMachine.add('NAVIGATE', Navigate(lambda ud: ud.params['pose']),
                           transitions = {'success': 'WAIT_HAND',
                                          'timeout': 'WAIT_HAND',
                                          'failure': 'WAIT_HAND'},
                           remapping = {'params': 'params'})

def create_sm():
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    with sm:
      smach.StateMachine.add('WAIT_HAND', WaitHandPushed(timeout=None,
                                                         threshold=12.,
                                                         prompt_msg="", success_msg=""),
                             transitions = {'success': 'COMMAND',
                                            'failure': 'WAIT_HAND',
                                            'timeout': 'WAIT_HAND'})
      def child_term_cb(outcome_map):
          if outcome_map['WAIT_COMMAND']:
              return True
          if outcome_map['WAIT_RELEASE']:
              return True
          return False
      cc = smach.Concurrence(outcomes=['success', 'failure'],
                             output_keys=['command'],
                             default_outcome='failure',
                             outcome_map={'success': {'WAIT_COMMAND':'success'}},
                             child_termination_cb=child_term_cb)
      with cc:
          smach.Concurrence.add('WAIT_RELEASE', WaitHandPushed(timeout=None,
                                                               threshold=-12.,
                                                               prompt_msg="", success_msg=""))
          smach.Concurrence.add('WAIT_COMMAND', GetCommand(get_command_sphinx,
                                                           parse_command),
                                remapping = {'command': 'command'})
      smach.StateMachine.add('COMMAND', cc,
                             transitions = {'success': 'PARSE_COMMAND',
                                            'failure': 'WAIT_HAND'})
      add_subsms()
    return sm

def get_command_sphinx(timeout):
    sphinx.set_jsgf(GRAMMAR)
    return sphinx.next_speech(timeout=timeout)

#rospy.init_node('command')

sm = create_sm()
iss = smach_ros.IntrospectionServer('command_sm', sm, '/COMMAND_SM_ROOT')
iss.start()
outcome = sm.execute()
rospy.spin()
iss.stop()
