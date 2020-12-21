#!/usr/bin/env python

import rospy
import smach

from common.smach_states import Navigate
import numpy as np

if __name__=='__main__':
	def move_gripper_fn(pose):
		from hsrb_interface import geometry
		position = geometry.Vector3(x = pose.pose.position.x,
									y = pose.pose.position.y,
									z = pose.pose.position.z)
		orientation = geometry.Quaternion(x = pose.pose.orientation.x,
										y = pose.pose.orientation.y,
										z = pose.pose.orientation.z,
										w = pose.pose.orientation.w)
		whole_body.move_end_effector_pose((position, orientation),
										pose.header.frame_id)

	def grasp_fn():
		gripper.grasp(-.01)

	import sys
	import re
	import hsrb_interface

	if len(sys.argv) < 2:
		print 'Usage: rosrun common smach_states.py <class_name> args...'
		quit()
	#rospy.init_node('smach_states')
	exp = [sys.argv[1], '(']
	exp.append(','.join(sys.argv[2:]))
	exp.append(')')
	exp = ''.join(exp)
	print 'state: '+exp
	if 'robot' in exp or 'whole_body' in exp or 'gripper' in exp:
		robot = hsrb_interface.Robot()
		whole_body = robot.get('whole_body')
		gripper = robot.get('gripper')
	else:
		rospy.init_node('smach_states')
	state = eval(exp)
	outcomes = state.get_registered_outcomes()
	sm = smach.StateMachine(outcomes=outcomes)
	with sm:
		smach.StateMachine.add('TEST', Navigate(lambda ud: ((0.0, 0.0), 1.0)), transitions={k:k for k in outcomes})
	print 'outcome: '+sm.execute()
	print 'userdata: {}'.format(dict(sm.userdata))
