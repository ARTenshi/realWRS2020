#!/usr/bin/lib python

import sys
import os
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
        def make_state_array(state_msg):
            result_array = []
            initial_state = list(set(state_msg.outcomes_from) ^
                                 set(state_msg.outcomes_to))
            initial_state.remove(state_msg.container_outcomes[0])

            c = np.array((state_msg.outcomes_from, state_msg.outcomes_to))
            result_array.append(initial_state[0])

            xy = np.where(c == initial_state)
            c[xy] = 'None'

            while not np.all(c == 'None'):
                result_array.append(c[xy[0]+1, xy[1]].tolist()[0])
                c[xy[0]+1, xy[1]] = 'None'
                xy = np.where(c == result_array[-1])
                c[xy] = 'None'
            print result_array
                
            return result_array

        try:
            rospy.loginfo('wait message')
            states_msg = rospy.wait_for_message('/server_name/smach/container_structure',
                                                SmachContainerStructure)
            state_array = make_state_array(states_msg)

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
        
        rospy.spin()

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
        print 'step_callback'
        now_state = msg.active_states[0]
        if now_state == None:
            return
        num = self.state_array.index(now_state)
        self.challenge_pub.publish(UInt32(num))
        
if __name__ == '__main__':
    rospy.init_node('bypass_vizbox')
    rosparam_path = sys.argv[1]
    bypass = BypassVizBox.initialize(challenge_name = 'debug challenge')
    
