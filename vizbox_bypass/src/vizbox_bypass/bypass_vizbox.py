#!/usr/bin/lib python
# -*- coding:utf-8 -*-

import sys
import os
import time

import yaml
import numpy as np

import rospy
import rosparam
from std_msgs.msg import String, UInt32, Bool
from vizbox_bypass.srv import EventStep
from smach_msgs.msg import SmachContainerStructure, SmachContainerStatus

class BypassVizBox:

    @staticmethod
    def initialize(challenge_name='Challenge'):
        def make_state_array(states_msg, structure_msg):
            result_array = []
            search_array = []
            already_search_array = []
            initial_state = states_msg.initial_states
            container_outcomes = structure_msg.container_outcomes
            result_array.append(initial_state[0])
            #search_array.append(initial_state[0])
            array = np.array((structure_msg.outcomes_from, structure_msg.outcomes_to))
            
            while not np.all(array == 'None'): # ここの条件をどうするか
                search_array = (list(set(result_array) ^  set(already_search_array)))
                y = np.array([])
                for j in search_array:
                    y = np.append(y, np.where(array[0] == j))
                if len(y) == 0: break
                print 'result array is ' + str(result_array)
                print 'search_array is'  + str(search_array)
                print 'array is ' + str(array)
                print 'y' + str(y)
                for wai in y:
                    now_search_state = array[0+1, int(wai)]
                    print 'now_search_state is ' + str(now_search_state)
                    
                    # 最終outcomeは追加しない     
                    if  any(i ==  now_search_state for i in container_outcomes):
                        print 'not append becaouse it is final outcome'
                        array[0+1, int(wai)] = None
                        array[0, int(wai)] = None
                        continue
                    
                    # すでに同じステートが追加されてたら追加しない
                    elif  any(k == now_search_state for k in result_array):
                        print 'not append becaouse it is already in result state'
                        array[0+1, int(wai)] = None
                        array[0, int(wai)] = None
                        continue
                    # searchにNoneがある可能性もあるからそれはパス
                    elif now_search_state == 'None':
                        print 'not append because it is None'
                        continue
                    else:
                        print 'append'
                        result_array.append(str(array[0+1, int(wai)]))
                        for h in search_array:
                            already_search_array.append(h)
                        array[0+1, int(wai)] = None
                        array[0, int(wai)] = None
                        
                print '------------------------------'
                    
            print 'last final array' + str(array)
            print result_array
            return result_array

        try:
            #rospy.loginfo('wait message')
            structure_msg = rospy.wait_for_message('/server_name/smach/container_structure',
                                                SmachContainerStructure)
            #rospy.loginfo('wait message 2')
            states_msg = rospy.wait_for_message('/server_name/smach/container_status',
                                                SmachContainerStatus)
            #rospy.loginfo('message arrive')
            state_array = make_state_array(states_msg, structure_msg)

            rospy.set_param('story/storyline', state_array)
            rospy.set_param('story/title', challenge_name)

            instance = BypassVizBox(state_array)
            return instance
        
        except:
            import traceback
            traceback.print_exc()
            
    def __init__(self, state_array):

        self.state_array = state_array
        self.step_count = 0
        
        self.sphinx_receiver = rospy.Subscriber('sphinx_topic', String, self.sphinx_cb)
        self.robot_speek = rospy.Subscriber('robot_speech_topic', String, self.speech_cb)
        self.smach_state = rospy.Subscriber('/server_name/smach/container_status',
                                            SmachContainerStatus, self.event_step_handller)

        #publisher for vizbox
        self.robot_pub = rospy.Publisher('robot_text', String, queue_size=1)
        self.speech_pub = rospy.Publisher('operator_text', String, queue_size=1)
        self.challenge_pub = rospy.Publisher('challenge_step', UInt32, queue_size=1)

        #rospy.Service('event_step', EventStep, self.event_step_handller)
        
        #rospy.spin()

    def sphinx_cb(self, msg):
        sphinx_word = String()
        sphinx_word.data = msg.data
        self.speech_pub.publish(sphinx_word)

    def speech_cb(self, msg):
        robot_speak = String()
        robot_speak.data = msg.data
        self.robot_pub.publish(robot_speak)
        
    '''
    def event_step_handller(self, srv):
        if srv.eventstep.data == 'forward':
            self.step_count+=1
            data = UInt32()
            data.data = self.step_count
        elif srv.eventstep.data == 'backward':
            if self.step_count == 0:
                data = UInt32()
                data.data = 0
                pass
            else:
                self.step_count-=1
                data = UInt32()
                data.data = self.step_count
        else:
            print 'please set forward or backwrd'

        self.challenge_pub.publish(data)
    '''
    
    def event_step_handller(self, msg):
        #print 'step_callback'
        now_state = msg.active_states[0]
        if now_state == 'None':
            return
        num = self.state_array.index(now_state)
        uint = UInt32()
        uint.data = num
        self.challenge_pub.publish(uint)
        
if __name__ == '__main__':
    rospy.init_node('bypass_vizbox')
    rosparam_path = sys.argv[1]
    bypass = BypassVizBox.initialize(challenge_name = 'debug challenge')
    
