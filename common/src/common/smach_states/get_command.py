#!/usr/bin/env python

import rospy
import smach
import traceback

from common import speech
from common.smach_states.utils import TemporarySubscriber


class GetCommand(smach.State):
    def __init__(self, listen_fn, parse_fn,
                 use_bypass=True, timeout=30., retry=2,
                 say_fn=None,
                 prompt_msg="Please give me a command.",
                 miss_msg="Sorry. I don't understand.",
                 bypass_msg="Could you give it to me by QR code?",
                 success_msg="I got it.",
                 qr_image_topic='/hsrb/head_rgbd_sensor/rgb/image_rect_color'):
        from cv_bridge import CvBridge
        import zbar

        smach.State.__init__(self, outcomes=['success', 'bypass', 'failure', 'timeout'],
                                   output_keys=['command'])
        self.timeout = timeout
        self.retry = retry
        self.listen_fn = listen_fn
        self.parse_fn = parse_fn
        self.use_bypass = use_bypass
        self.bridge = CvBridge()
        self.scanner = zbar.ImageScanner()
        self.scanner.parse_config('enable')
        self.qr_data = []
        self.prompt_msg = prompt_msg
        self.miss_msg = miss_msg
        self.bypass_msg = bypass_msg
        self.success_msg = success_msg
        self.qr_image_topic = qr_image_topic
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say

    def execute(self, userdata):
        try:
            self.qr_data = []
            from sensor_msgs.msg import Image
            with TemporarySubscriber(self.qr_image_topic, Image, self.image_cb):
                count = 0
                while count < self.retry:
                    if self.use_bypass and self.qr_data:
                        parse = filter(self.parse_fn, self.qr_data)
                        if parse:
                            if self.say_fn and self.success_msg:
                                self.say_fn(self.success_msg)
                            userdata.command = parse[0]
                            return 'success'
                    if self.say_fn and self.prompt_msg:
                        self.say_fn(self.prompt_msg)
                    try:
                        text = self.listen_fn(self.timeout)
                    except:
                        rospy.logerr(traceback.format_exc())
                        if self.preempt_requested():
                            return 'timeout'
                        if not self.use_bypass:
                            return 'timeout'
                        break
                    if self.parse_fn(text):
                        if self.say_fn and self.success_msg:
                            self.say_fn(self.success_msg)
                        userdata.command = text
                        return 'success'
                    if self.say_fn and self.miss_msg:
                        self.say_fn(self.miss_msg)
                    rospy.sleep(2)
                    count += 1

                if not self.use_bypass:
                    return 'timeout'

                if self.say_fn and self.bypass_msg:
                    self.say_fn(self.bypass_msg)

                elapsed = 0.
                while not rospy.is_shutdown() and (not self.timeout or elapsed < self.timeout):
                    parse = filter(self.parse_fn, self.qr_data)
                    if parse:
                        if self.say_fn and self.success_msg:
                            self.say_fn(self.success_msg)
                        userdata.command = parse[0]
                        return 'success'
                    rospy.sleep(.2)
                    elapsed += .2
            return 'timeout'
        except:
            rospy.logerr(traceback.format_exc())
            return 'failure'

    def image_cb(self, msg):
        import PIL.Image
        import zbar

        img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        pimg = PIL.Image.fromarray(img)
        zimg = zbar.Image(img.shape[1], img.shape[0],
                          'Y800', pimg.tobytes())
        if self.scanner.scan(zimg):
	    print 'qr true'
            self.qr_data = [symbol.data for symbol in zimg]
