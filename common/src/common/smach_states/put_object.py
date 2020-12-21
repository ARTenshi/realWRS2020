#!/usr/bin/env python

import rospy
import smach
import traceback

from depth_lib import gripper_pose

from common import speech, rospose_to_tmcpose


class PutObject(smach.State):
    def __init__(self, robot,
                 say_fn=None,
                 start_msg="",
                 success_msg="",
                 tf_buffer=None):
        from common import HSRB_Impedance
        smach.State.__init__(self, outcomes=['success', 'failure'],
                                   input_keys=['placement'])
        self.start_msg = start_msg
        self.success_msg = success_msg
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.whole_body = robot.get('whole_body')
        self.gripper = robot.get('gripper')
        if rospy.get_param('is_sim', False) == False:
            self.impedance = HSRB_Impedance()
        import tf2_ros
        import tf2_geometry_msgs
        if tf_buffer is None:
            self.tf_buf = tf2_ros.Buffer(rospy.Duration(5.))
            tf2_ros.TransformListener(self.tf_buf)
        else:
            self.tf_buf = tf_buffer

    def execute(self, userdata):
        try:
            if self.say_fn and self.start_msg:
                self.say_fn(self.start_msg)
            self.impedance.move_to_neutral()

            pos = self.tf_buf.transform(userdata.placement, 'base_footprint',
                                        rospy.Duration(1.))
            poses = gripper_pose((pos.point.x, pos.point.y, pos.point.z), (0, -1, 0), (1, 0, 0), back_dist=.3)
            pose1, pose2 = map(rospose_to_tmcpose, poses)
            self.whole_body.move_end_effector_pose(pose1, 'base_footprint')
            self.whole_body.move_end_effector_pose(pose2, 'hand_palm_link')
            self.gripper.grasp(0.01)
            pose3 = (pose2[0][0], pose2[0][1], -pose2[0][2]), pose2[1]
            self.whole_body.move_end_effector_pose(pose3, 'hand_palm_link')

            if self.say_fn and self.success_msg:
                self.say_fn(self.success_msg)
            return 'success'
        except:
            rospy.logerr(traceback.format_exc())
            return 'failure'
