#!/usr/bin/env python

import rospy
import smach
import traceback

from common import speech
from common.smach_states.utils import TemporarySubscriber

# Waits the gripper to be pushed down.
# If threshold < 0, this state waits the gripper to be released.


class WaitHandPushed(smach.State):
    def __init__(self, threshold=12., timeout=120.,
                 say_fn=None,
                 prompt_msg="Please push down my hand.",
                 success_msg="OK."):
        smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'])
        self.threshold = threshold
        self.pushed = False
        self.current_value = None
        self.timeout = timeout
        #self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.prompt_msg = prompt_msg
        self.success_msg = success_msg

    def execute(self, userdata):
        if rospy.get_param('is_sim', False):
            #self.say_fn(
            #    'It is the simulation mode. Pass the "Wait-Hand-Pushed" state.')
            return 'success'
        try:
            from geometry_msgs.msg import WrenchStamped
            self.pushed = False
            self.current_value = None
            #if self.say_fn and self.prompt_msg:
            #    self.say_fn(self.prompt_msg)

            with TemporarySubscriber('/hsrb/wrist_wrench/raw', WrenchStamped, self.wrench_cb):
                t = 0.
                while not rospy.is_shutdown() and not self.pushed:
                    rospy.sleep(1.)
                    t += 1.
                    if self.timeout is not None and t > self.timeout:
                        break
                    rospy.loginfo('Waiting for the hand to be pushed. value={}, threshold={}'.format(
                        self.current_value, self.threshold))
            if self.pushed:
                #if self.say_fn and self.success_msg:
                #    self.say_fn(self.success_msg)
                return 'success'
            return 'timeout'
        except:
            rospy.logerr(traceback.format_exc())
            return 'failure'

    def wrench_cb(self, msg):
        self.current_value = msg.wrench.force.x
        if self.threshold > 0.:
            if self.current_value > self.threshold:
                self.pushed = True
        else:
            if self.current_value < -self.threshold:
                self.pushed = True
