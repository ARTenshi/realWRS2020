#!/usr/bin/env python
import rospy
import tf
import common.location_dict as ARENA
import math

if __name__ == '__main__':
    rospy.init_node('tf_locations')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)
    profile = ARENA.LOCATIONS
    #f_profile = ARENA.FIRST_POSITION
    #print profile
    while not rospy.is_shutdown():
        for k, v in profile.items():
            #print k, v[0][0], v[0][1], v[1]
            #print k, v[0][0], v[0][1], v[1]
            #while not rospy.is_shutdown():
            br.sendTransform((v[0][0],v[0][1],.0),
                            (.0,.0,math.sin(v[1]/2),math.cos(v[1]/2)),
                            rospy.Time.now(),
                            k,
                            'map')
        rate.sleep()
                            
