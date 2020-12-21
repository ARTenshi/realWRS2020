#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Class for image detection
Author: yamakawa
"""
import colorsys
import cv2
import IPython
import matplotlib.pyplot as plt
import numpy as np
from autolab_core import Box
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image, CameraInfo
from yolo_tf.msg import ObjectArray
from yolo_tf.libs import TreeReader
import message_filters

COLORS_ = [(255, 0, 0), (0, 255, 0), (0, 0, 255),
           (255, 255, 0), (255, 0, 255), (0, 255, 255),
           (127, 0, 0), (0, 127, 0), (0, 0, 127),
           (127, 127, 0), (127, 0, 127), (0, 127, 127),
           (127, 127, 127)]
           
# for unknonw (general) object name
# TR_unknown = TreeReader("get_names")
# for known object name
TR_known = TreeReader("get_names_known")

class MyDetection(object):
    """
        Detect objects using yolo from HSR camera images.
    """

    def __init__(self):

        #rospy.init_node("detection_view_node")

        self._bridge = CvBridge()
        self._color_input_image = None
	self.color_image_msg = None
        self._depth_input_image = None
	self.depth_image_msg = None
        self.output_image = None

	self.bb_list = []
	self.obj_list = []

        color_topic_name = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        self._color_image_sub = message_filters.Subscriber(color_topic_name, Image)
        #rospy.wait_for_message(topic_name, Image, timeout=5.0)
        depth_topic_name = '/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw'
        self._depth_image_sub = message_filters.Subscriber(depth_topic_name, Image)
        #rospy.wait_for_message(topic_name, Image, timeout=5.0)

        camera_topic_name = '/hsrb/head_rgbd_sensor/rgb/camera_info'
        self._camera_info_sub = rospy.Subscriber(
            camera_topic_name, CameraInfo, self._cam_info_cb)
        rospy.wait_for_message(camera_topic_name, CameraInfo, timeout=5.0)

        topic_name = '/yolo2_image_node/image_result'
        self._result_image_pub = rospy.Publisher(
            topic_name, Image, queue_size=10)

        yolo_topic_name = '/known_objects'
        self._result_sub = message_filters.Subscriber(
            yolo_topic_name, ObjectArray)
        #rospy.wait_for_message(topic_name, ObjectArray, timeout=5.0)

        self._all_sub = message_filters.ApproximateTimeSynchronizer(
            [self._color_image_sub, self._depth_image_sub, self._result_sub], 30, 1)
        self._all_sub.registerCallback(self._callback)

        rospy.wait_for_message(yolo_topic_name, ObjectArray, timeout=15.0)
        rospy.wait_for_message(color_topic_name, Image, timeout=15.0)
        rospy.wait_for_message(depth_topic_name, Image, timeout=15.0)

    def _cam_info_cb(self, data):
        """
        get camera info
        """
        self.camera_info = data

    def _color_image_cb(self, data):
        """
        get color image from HSR head camera
        """
        try:
            self.width = float(data.width)
            self.height = float(data.height)
            self.encoding = data.encoding
            self.color_image_msg = data
            self._color_input_image = self._bridge.imgmsg_to_cv2(
                data, self.encoding)
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

    def _depth_image_cb(self, data):
        """
        get depth image from HSR head sensor
        """
        try:
            self.depth_image_msg = data
            self._depth_input_image = self._bridge.imgmsg_to_cv2(
                data).astype(np.uint8)
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

    def _result_cb(self, data):
        """
        Make detection from color image.
        Receive yolo's detection result.
        Return color + depth images and BBoxes of objects.
        """
        if self._color_input_image is None:
            print "input image is None"
            return
        self.output_image = self._color_input_image.copy()
        detect_classes = []
        self.bb_list = []
        self.obj_list = []

        self.data = data

        for num, detection in enumerate(data.objects):
            self.trace = TR_known.trace_max(detection.class_probability)

            # skip detection with low probability
            #if self.trace[-2][-1] < 1e-20:
            #    continue
            

            Xmin = detection.left
            Ymin = detection.top
            Xmax = detection.right
            Ymax = detection.bottom

            self.bb_list.append([Xmin, Xmax, Ymin, Ymax])

            self.obj_list.append(self.trace)

            cv2.rectangle(self.output_image,
                          (int(Xmin), int(Ymin)),
                          (int(Xmax), int(Ymax)),
                          COLORS_[detection.box_index % len(COLORS_)], 2)
        
            obj_name = self.trace[0][1]  #self.trace[-1][1] + ", " + self.trace[-2][1]
            cv2.putText(self.output_image, obj_name,
                        (int(Xmin), int(Ymin)), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        COLORS_[detection.box_index % len(COLORS_)], 2)

        self._result_image_pub.publish(
            self._bridge.cv2_to_imgmsg(
                self.output_image, encoding=self.encoding))
        #cv2.imshow("yolo_tf", self.output_image)
        #cv2.waitKey(2)
        
    def cv_show(self):
        cv2.imshow("yolo_tf", self.output_image)
        cv2.waitKey(1)

    def _callback(self, color_image, depth_image, result):
        self. _color_image_cb(color_image)
        self. _depth_image_cb(depth_image)
        self. _result_cb(result)

    def detection(self):
        """
        Return detection result, color+depth images.
        """
        if self.color_image_msg is None:
             rospy.wait_for_message('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image, timeout=15.0)

        return self.output_image, self.color_image_msg, self.depth_image_msg, self.bb_list, self.obj_list

    def get_depth_image(self):
        """
        Return depth image
        """
        return self._depth_input_image

    def get_camera_info(self):
        """
        Return camera info
        """
        return self.camera_info

