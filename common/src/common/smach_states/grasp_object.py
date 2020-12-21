#!/usr/bin/env python

import rospy
import smach
import traceback

from common import speech
from common import weight_calculator


class GraspObject(smach.State):
    def __init__(self, robot,
                 say_fn=None,
                 start_msg="",
                 success_msg="",
                 weight_measurement=False):
        smach.State.__init__(self, outcomes=['success', 'failure'],
                                   input_keys=['object'],
                                   output_keys=['weight', 'default_weight'])
        self.start_msg = start_msg
        self.success_msg = success_msg
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.whole_body = robot.get('whole_body')
        self.gripper = robot.get('gripper')
        self.weight_calculator = None
        if weight_measurement is True:
            self.weight_calculator = weight_calculator.WeightCalculator()

    def execute(self, userdata):
        try:
            if self.say_fn and self.start_msg:
                self.say_fn(self.start_msg)

            from hsrb_interface import geometry
            pose = userdata.object.approachPose.pose
            pos = geometry.Vector3(x=pose.position.x,
                                   y=pose.position.y,
                                   z=pose.position.z)
            ori = geometry.Quaternion(x=pose.orientation.x,
                                      y=pose.orientation.y,
                                      z=pose.orientation.z + 0.1,
                                      w=pose.orientation.w)
            frame_id = userdata.object.approachPose.header.frame_id
            self.whole_body.move_end_effector_pose((pos, ori), frame_id)
            pose = userdata.object.graspPose.pose
            pos = geometry.Vector3(x=pose.position.x,
                                   y=pose.position.y,
                                   z=pose.position.z)
            ori = geometry.Quaternion(x=pose.orientation.x,
                                      y=pose.orientation.y,
                                      z=pose.orientation.z,
                                      w=pose.orientation.w)
            frame_id = userdata.object.graspPose.header.frame_id
            self.gripper.command(1.)
            userdata.weight = 0.
            userdata.default_weight = 0.
            if self.weight_calculator is not None:
                rospy.sleep(1.)
                self.weight_calculator.start_subscriber()
                self.gripper.grasp(-0.01)
                rospy.sleep(1.)
                self.weight_calculator.end_subscriber()
                userdata.default_weight = self.weight_calculator.get_gram_weight()
                self.gripper.command(1.)
                rospy.sleep(1.)
                self.weight_calculator.start_subscriber()
            self.whole_body.move_end_effector_pose((pos, ori), frame_id)
            self.gripper.grasp(-0.01)
            pos = geometry.Vector3(-pos.x, -pos.y, -pos.z)
            self.whole_body.move_end_effector_pose((pos, ori), frame_id)
            if self.weight_calculator is not None:
                self.weight_calculator.end_subscriber()
                rospy.sleep(1.)
                userdata.weight = self.weight_calculator.get_gram_weight()

            if self.say_fn and self.success_msg:
                self.say_fn(self.success_msg)
            return 'success'
        except:
            rospy.logerr(traceback.format_exc())
            return 'failure'
