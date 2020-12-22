#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from yolo_tf.msg import ObjectArray, FeatureArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber
from yolo_tf.libs import TreeReader

bridge = CvBridge()
tr = TreeReader('get_names_known')
print "a"

def callback(image, objects):
    print 'callback'
    try:
        cv_image = bridge.imgmsg_to_cv2(image, 'bgr8')
        print "b"
    except CvBridgeError as e:
        rospy.logerr(e)

    for obj in objects.objects:
        #rospy.loginfo('objectness={}'.format(obj.objectness))
        if obj.objectness < 0.35:
            print "c"
            continue
        name = None
        prob = 0.
        for _, _name, _prob in tr.trace_max(obj.class_probability):
            if _prob > 0.1:
                print "oppai"
                name = _name
                prob = _prob
                print name, prob
            else:
                break

        cv2.rectangle(cv_image, (int(obj.left),int(obj.top)), (int(obj.right),int(obj.bottom)), (0,255,0), 3)
        cv2.putText(cv_image, name, (int(obj.left),int(obj.top)), cv2.FONT_HERSHEY_PLAIN, 1., (0,0,255))
        cv2.putText(cv_image, 'indices=({},{},{})'.format(obj.row, obj.column, obj.box_index),
                    (int(obj.left),int(obj.top+20)), cv2.FONT_HERSHEY_PLAIN, 1., (0,0,255))
        cv2.putText(cv_image, 'confidence={}'.format(obj.objectness*prob),
                    (int(obj.left),int(obj.top+40)), cv2.FONT_HERSHEY_PLAIN, 1., (0,0,255))

    cv_image = cv2.resize(cv_image, (480,480))
    cv2.imshow(image_sub.resolved_name, cv_image)
    cv2.waitKey(1)
    
rospy.init_node('yolo9000_visualizer')
image_sub = Subscriber('image', Image)
object_sub = Subscriber('known_objects', ObjectArray)
sub = ApproximateTimeSynchronizer([image_sub, object_sub], 10, 10)
sub.registerCallback(callback)
print 'TreeReader created'
rospy.spin()