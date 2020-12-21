#!/usr/bin/env python

import rospy
import smach
import traceback

from common import speech
from common.smach_states.utils import TemporarySubscriber


class WaitDoorOpen(smach.State):
    def __init__(self, threshold=1.5, timeout=120.,
                 say_fn=None,
                 knock_msg="Knock knock.", success_msg="Thank you.",
                 depth_topic='/hsrb/head_rgbd_sensor/depth_registered/image'):
        from cv_bridge import CvBridge
        smach.State.__init__(self, outcomes=['success', 'timeout', 'failure'])
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.door_is_open = False
        self.current_value = None
        self.threshold = threshold
        self.bridge = CvBridge()
        self.timeout = timeout
        self.knock_msg = knock_msg
        self.success_msg = success_msg
        self.depth_topic = depth_topic

    def execute(self, userdata):
        try:
            from sensor_msgs.msg import Image
            self.door_is_open = False
            self.current_value = None
            if self.say_fn and self.knock_msg:
                self.say_fn(self.knock_msg)

            with TemporarySubscriber(self.depth_topic, Image, self.depth_cb):
                t = 0.
                while not rospy.is_shutdown() and not self.door_is_open:
                    rospy.sleep(1.)
                    t += 1.
                    if t > self.timeout:
                        break
                    rospy.loginfo('Waiting for the door to open. value={}, threshold={}'.format(
                        self.current_value, self.threshold))
            if self.door_is_open:
                if self.say_fn and self.success_msg:
                    self.say_fn(self.success_msg)
                return 'success'
            return 'timeout'
        except:
            rospy.logerr(traceback.format_exc())
            return 'failure'

    def depth_cb(self, msg):
        import numpy as np
        img = self.bridge.imgmsg_to_cv2(msg)
        self.current_value = img[np.isfinite(img)].mean()
        if self.current_value > self.threshold:
            self.door_is_open = True
