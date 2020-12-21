#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import traceback

from vizbox_bypass import BypassVizBox

##################################################

# Main
rospy.loginfo('initializing...')
rospy.sleep(3)

def create_sm():

  sm = smach.StateMachine(outcomes=['success', 'failure'])

  with sm:

        @smach.cb_interface(outcomes=['success', 'timeout', 'failure'])
        def aaaa(self):
            try:
                if a:
                    return 'timeout'

                return 'success'
            except:
                return 'failure'
                
        
	smach.StateMachine.add('WAIT_HAND', smach.CBState(aaaa),
				transitions={'success':'TESTSTATE',
				             'timeout':'TESTSTATE2',
				             'failure':'failure'})

	@smach.cb_interface(outcomes=['success', 'failure'])
	def test_state_cb(self):
		try:
			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('TESTSTATE', smach.CBState(test_state_cb),
				transitions = {'success': 'success',
					       'failure': 'failure'})

        smach.StateMachine.add('TESTSTATE2', smach.CBState(test_state_cb),
                               transitions = {'success':'TESTSTATE',
                                              'failure':'failure'})
        

  return sm

rospy.init_node('test')
sm = create_sm()

###code for vizbox
sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
sis.start()
viz_ins = BypassVizBox.initialize('test code challenge')
###

rospy.spin()
outcome = sm.execute()

if outcome == 'success':
	#rospy.sleep(2)
	rospy.loginfo('I finished the task.')
else:
	rospy.signal_shutdown('Some error occured.')
