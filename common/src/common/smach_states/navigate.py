#!/usr/bin/env python

import rospy
import smach
import traceback
import types
from contextlib import nested

from common import speech

class Navigate(smach.State):
    def __init__(self, goal_pose,
                 ref_frame='map', timeout=120.,
		 say_fn=None,
		 start_msg="",
		 timeout_msg="",
		 success_msg="",
		 contexts=[],
		 spin_fn=None,
		 input_keys=[], additional_outcomes=[], output_keys=[]):
        import actionlib
	from move_base_msgs.msg import MoveBaseAction
	smach.State.__init__(self, outcomes=['success', 'failure', 'timeout']+additional_outcomes,
			     input_keys=input_keys,
                             output_keys=output_keys)
	self.goal_pose = goal_pose
	self.ref_frame = ref_frame
	self.timeout = timeout
	self.start_msg = start_msg
	self.timeout_msg = timeout_msg
	self.success_msg = success_msg
	self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
	self.contexts = contexts
	if spin_fn is not None:
	    self.spin_fn = spin_fn
	else:
	    rate = rospy.Rate(1.)
	    self.spin_fn = lambda ud: rate.sleep()
	self.client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
	self.client.wait_for_server(rospy.Duration(1.))

    def execute(self, userdata):
        try:
	    from move_base_msgs.msg import MoveBaseGoal
	    import math
	    import actionlib

	    if isinstance(self.goal_pose, types.FunctionType):
		goal_pose = self.goal_pose(userdata)
	    else:
		goal_pose = self.goal_pose
            if isinstance(self.ref_frame, types.FunctionType):
		ref_frame = self.ref_frame(userdata)
	    else:
		ref_frame = self.ref_frame
            rospy.loginfo('Moving to {}, {}'.format(goal_pose[0], goal_pose[1]))

	    if self.say_fn and self.start_msg:
		try:
		    msg = self.start_msg.format(**userdata)
		    self.say_fn(msg)
		except:
		    rospy.logerr(traceback.format_exc())

	    goal = MoveBaseGoal()
	    goal.target_pose.header.frame_id = ref_frame
	    goal.target_pose.pose.position.x = goal_pose[0][0]
	    goal.target_pose.pose.position.y = goal_pose[0][1]
	    goal.target_pose.pose.orientation.z = math.sin(goal_pose[1]/2)
	    goal.target_pose.pose.orientation.w = math.cos(goal_pose[1]/2)
	    self.client.send_goal(goal)
	    timeout = rospy.Duration(self.timeout) if self.timeout else None
	    with nested(*self.contexts):
		while not rospy.is_shutdown():
		    state = self.client.get_state()
		    rospy.loginfo('MoveBaseAction state: '+actionlib.GoalStatus.to_string(state))
		    if state != actionlib.GoalStatus.PENDING and state != actionlib.GoalStatus.ACTIVE:
			break
                    outcome = self.spin_fn(userdata)
		    if isinstance(outcome, str):
			self.client.cancel_goal()
			return outcome

	    if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
		if self.say_fn and self.success_msg:
		    try:
			msg = self.success_msg.format(**userdata)
			self.say_fn(msg)
		    except:
			rospy.logerr(traceback.format_exc())
		return 'success'
            if self.say_fn and self.timeout_msg:
		try:
		    msg = self.timeout_msg.format(**userdata)
		    self.say_fn(msg)
		except:
		    rospy.logerr(traceback.format_exc())
            self.client.cancel_goal()
	    return 'timeout'
	except:
	    rospy.logerr(traceback.format_exc())
	    return 'failure'
