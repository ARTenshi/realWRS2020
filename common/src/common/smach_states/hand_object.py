#!/usr/bin/env python

import rospy
import smach
import traceback

from common import speech
from common.smach_states.utils import TemporarySubscriber


class HandObject(smach.State):
    def __init__(self, robot,
                 threshold=10.,
                 timeout=10.,
                 say_fn=None,
                 start_msg="Here you are.",
                 success_msg="",
                 tf_buffer=None):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'failure'],
                                   input_keys=['person'])
        self.start_msg = start_msg
        self.success_msg = success_msg
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.whole_body = robot.get('whole_body')
        self.gripper = robot.get('gripper')
        self.timeout = timeout
        self.threshold = threshold
        self.pulled = False
        self.current_value = None
        self.initial_force = None
        import tf2_ros
        import tf2_geometry_msgs
        if tf_buffer is None:
            self.tf_buf = tf2_ros.Buffer(rospy.Duration(5.))
            tf2_ros.TransformListener(self.tf_buf)
        else:
            self.tf_buf = tf_buffer

    def execute(self, userdata):
        try:
            import math
            from geometry_msgs.msg import WrenchStamped
            self.pulled = False
            self.current_value = None

            from hsrb_interface import geometry
            frame_id = 'base_footprint'
            point = self.tf_buf.transform(userdata.person.center, frame_id,
                                          rospy.Duration(1.))
            dx = point.point.x
            dy = point.point.y
            d = math.sqrt(dx**2 + dy**2)
            dx /= d
            dy /= d
            pos = geometry.Vector3(x=point.point.x - dx*.3,
                                   y=point.point.y - dy*.3,
                                   z=point.point.z)
            ori = self.whole_body.get_end_effector_pose()[1]
            self.whole_body.move_end_effector_pose((pos, ori), frame_id)

            self.initial_force = rospy.wait_for_message(
                '/hsrb/wrist_wrench/raw', WrenchStamped).wrench.force

            if self.say_fn and self.start_msg:
                self.say_fn(self.start_msg)

            with TemporarySubscriber('/hsrb/wrist_wrench/raw', WrenchStamped, self.wrench_cb):
                t = 0.
                while not rospy.is_shutdown() and not self.pulled:
                    rospy.sleep(.1)
                    t += .1
                    if t > self.timeout:
                        break
                    rospy.loginfo('Waiting for the person to pull the object. value={}, threshold={}'.format(
                        self.current_value, self.threshold))
            if self.pulled:
                if self.say_fn and self.success_msg:
                    self.say_fn(self.success_msg)
                self.gripper.command(0.9)
                # self.gripper.grasp(-0.01)
                return 'success'
            return 'timeout'
        except:
            rospy.logerr(traceback.format_exc())
            return 'failure'

    def wrench_cb(self, msg):
        import math
        f = msg.wrench.force
        d = (f.x-self.initial_force.x)**2 + \
            (f.y-self.initial_force.y)**2 + (f.z-self.initial_force.z)**2
        self.current_value = math.sqrt(d)
        if self.current_value > self.threshold:
            self.pulled = True
