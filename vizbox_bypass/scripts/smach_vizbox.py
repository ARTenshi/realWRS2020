#!/usr/bin/env python

import rospy
import smach
import smach_ros

from vizbox_bypass import BypassVizBox




class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.sleep(1)
        return 'success'

class Bar2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR2')
        rospy.sleep(1)
        return 'success'

    
class Bar3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR3')
        rospy.sleep(1)
        return 'success'
    
    
def main():
    rospy.init_node('smach_example_state_machine')

    sm = smach.StateMachine(outcomes=['success'])

    with sm:
        smach.StateMachine.add('waithand', Bar(), 
                               transitions={'success':'command'})
        smach.StateMachine.add('command', Bar2(), 
                               transitions={'success':'move'})
        smach.StateMachine.add('move', Bar3(), 
                               transitions={'success':'success'})


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    viz_ins = BypassVizBox.initialize('aaa challenge')
    
    outcome = sm.execute()
    print 'fin'
    rospy.spin()


if __name__ == '__main__':
    main()
