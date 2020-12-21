#!/usr/bin/env python

import rospy
import smach
import traceback
import types
import copy
import cv2
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

from common import speech
from common.smach_states.utils import TemporarySubscriber, default_accept_object_fn, transform_geometry_pose_stamped, transform_geometry_point_stamped
from depth_lib import depth_lib2
from depth_lib.srv._GetObject import GetObject
from depth_lib.srv._GetTableTopObject import GetTableTopObject
from yolo_tf.msg import ObjectArray
from yolo_tf.srv import DetectObjects


class FindObject(smach.State):
    # yolo_9000_cls may be:
    # - None (any class accepted, threshold is for objectness)
    # - int value (class ID, threshold is probability of the class)
    # - array of int (class IDs, threshold must be an array of the same length)
    # - function that takes userdata and returns one of above
    # threshold may be a function as well
    def __init__(self, yolo9000_cls, threshold,
                 input_keys=[],
                 obj_topic='objects',
                 get_object_srv='get_object',
                 get_object_srv_type=GetObject,
                 get_object_srv_kwarg={},
                 get_object_trials=3,
                 accept_object_fn=default_accept_object_fn,
                 names_srv='get_names',
                 timeout=30.,
                 say_fn=None,
                 start_msg="Looking for an object.",
                 timeout_msg="",
                 success_msg="Found an object.",
                 camera_joints=[{'lift': 0., 'tilt': 0., 'pan': 0.},
                                {'lift': 0., 'tilt': 0., 'pan': -.8},
                                {'lift': 0., 'tilt': 0., 'pan': .8}],
                 tf_buffer=None,
                 output_keys=[],
                 detect_objects_srv='detect_objects',
                 debug_image_topic=None,
                 debug_cv2_image_save_dir=None,
                 camera_joints_execution=False):
        from common import HSRB_Xtion
        smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
                                   input_keys=input_keys,
                                   output_keys=['object', 'objects', 'yolo_object', 'yolo_objects', 'object_map', 'objects_map', 'point_cloud2'] + output_keys)
        self.yolo9000_cls = yolo9000_cls
        self.threshold = threshold
        self.obj_topic = obj_topic
        self.timeout = timeout
        self.start_msg = start_msg
        self.timeout_msg = timeout_msg
        self.success_msg = success_msg
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.found_depth_object_map = None
        self.found_depth_objects_map = []
        self.found_depth_object = None
        self.found_depth_objects = []
        self.found_yolo_object = None
        self.found_yolo_objects = []
        self.found_point_cloud2 = None
        self.found_class = None
        from yolo_tf.libs import TreeReader
        self.tr = TreeReader(names_srv)
        if get_object_srv_type is None:
            get_object_srv_type = GetObject
        rospy.wait_for_service(get_object_srv)
        self.get_object = rospy.ServiceProxy(
            get_object_srv, get_object_srv_type)
        self.get_object_srv_kwarg = get_object_srv_kwarg
        self.get_object_trials = get_object_trials
        if accept_object_fn:
            self.accept_object_fn = accept_object_fn
        else:
            self.accept_object_fn = lambda ud, obj: True
        self.xtion = HSRB_Xtion(tf_buffer)
        self.camera_joints = camera_joints
        self.tf_buffer = tf_buffer
        self.depth_lib = depth_lib2.DepthLib(tf_buffer)
        if get_object_srv_type == GetTableTopObject:
            self.get_object = self.depth_lib.get_table_top_object
        else:
            self.get_object = self.depth_lib.get_object
        self.point_cloud2_sub = None
        rospy.wait_for_service(detect_objects_srv)
        self.detect_objects = rospy.ServiceProxy(
            detect_objects_srv, DetectObjects)
        self.cv_bridge = CvBridge()
        self.debug_cv2_image_save_dir = debug_cv2_image_save_dir
        if debug_image_topic:
            self.debug_image_pub = rospy.Publisher(
                debug_image_topic, Image, queue_size=1)
        else:
            self.debug_image_pub = None
        if camera_joints_execution:
            rospy.logwarn('Argument "camera_joints_execution" is not used.')
        self.force_camera_joints_execution = camera_joints_execution

    def execute(self, userdata):
        try:
            if isinstance(self.yolo9000_cls, types.FunctionType):
                self._yolo9000_cls = self.yolo9000_cls(userdata)
            else:
                self._yolo9000_cls = self.yolo9000_cls
            if isinstance(self._yolo9000_cls, int) or isinstance(self._yolo9000_cls, str):
                self._yolo9000_cls = [self._yolo9000_cls]
            if isinstance(self.threshold, types.FunctionType):
                self._threshold = self.threshold(userdata)
            else:
                self._threshold = self.threshold
            if isinstance(self._threshold, float):
                self._threshold = [self._threshold]
            if isinstance(self.get_object_srv_kwarg, types.FunctionType):
                self._get_object_srv_kwarg = self.get_object_srv_kwarg(userdata)
            else:
                self._get_object_srv_kwarg = self.get_object_srv_kwarg
            if isinstance(self.camera_joints, types.FunctionType):
                self.camera_joints = self.camera_joints(userdata)
            else:
                self.camera_joints = self.camera_joints

            if self.say_fn and self.start_msg:
                try:
                    msg = self.start_msg.format(**userdata)
                    self.say_fn(msg)
                except:
                    rospy.logerr(traceback.format_exc())

            userdata.objects_map = []
            userdata.object_map = None
            userdata.objects = []
            userdata.object = None
            userdata.yolo_objects = []
            userdata.yolo_object = None
            userdata.point_cloud2 = None
            self.found_depth_object_map = None
            self.found_depth_objects_map = []
            self.found_depth_object = None
            self.found_depth_objects = []
            self.found_yolo_object = None
            self.found_yolo_objects = []
            self.found_point_cloud2 = None
            self._userdata = userdata
            joint_ind = 0
            t0 = rospy.Time.now()
            t1 = t0
            while not rospy.is_shutdown():
                try:
                    self.__point_cloud2_cb(rospy.wait_for_message(
                        '/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2))
                except:
                    rospy.logerr('Failed to process point cloud.')

                if self.found_depth_object is not None:
                    break
                if self.camera_joints:
                    _t = (rospy.Time.now() - t1).to_sec()
                    if _t > 5.:
                        print self.camera_joints[joint_ind]
                        try:
                            arm_result, head_result = self.xtion.move(
                                wait=True, **self.camera_joints[joint_ind])
                        except:
                            arm_result, head_result = False, False
                        if arm_result and head_result:
                            joint_ind = (joint_ind+1) % len(self.camera_joints)
                            t1 = rospy.Time.now()
                t = (rospy.Time.now() - t0).to_sec()
                if t > self.timeout:
                    if self.say_fn and self.timeout_msg:
                        try:
                            msg = self.timeout_msg.format(**userdata)
                            self.say_fn(msg)
                        except:
                            rospy.logerr(traceback.format_exc())
                    return 'timeout'
            userdata.objects_map = self.found_depth_objects_map
            userdata.object_map = self.found_depth_object_map
            userdata.objects = self.found_depth_objects
            userdata.object = self.found_depth_object
            userdata.yolo_objects = self.found_yolo_objects
            userdata.yolo_object = self.found_yolo_object
            userdata.point_cloud2 = self.found_point_cloud2
            if self.say_fn and self.success_msg:
                try:
                    msg = self.success_msg.format(**userdata)
                    self.say_fn(msg)
                except:
                    rospy.logerr(traceback.format_exc())
            return 'success'
        except:
            rospy.logerr(traceback.format_exc())
            return 'failure'

    def check_object_acceptance(self, obj):
        depth_obj = None
        for _ in xrange(self.get_object_trials):
            try:
                depth_obj = self.get_object(top=obj.top,
                                            left=obj.left,
                                            bottom=obj.bottom,
                                            right=obj.right,
                                            **self._get_object_srv_kwarg)
            except:
                rospy.logerr(traceback.format_exc())
                continue
            if depth_obj is not None and self.accept_object_fn(self._userdata, depth_obj):
                rospy.loginfo('Object accepted by accept_object_fn.')
                break
            else:
                depth_obj = None
        return depth_obj

    def __point_cloud2_cb(self, pointmsg):
        #print('__point_cloud2_cb() start')
        self.depth_lib.set_pointmsg(pointmsg)
        __points, __rgbimg = depth_lib2.read_point_cloud(pointmsg)
        __imgmsg = self.cv_bridge.cv2_to_imgmsg(__rgbimg, 'rgb8')

        try:
            objmsg = self.detect_objects(image=__imgmsg)
        except:
            rospy.logerr(traceback.format_exc())
            #print('__point_cloud2_cb() exception')
            return

        max_prob = 0.
        max_score = -1.
        max_cls = -1.
        max_yolo_obj = None
        yolo_obj_list = []
        max_depth_obj = None
        depth_obj_list = []
        max_depth_obj_map = None
        depth_obj_map_list = []
        obj_thresh = 0.1
        if not self._yolo9000_cls and self._threshold:
            obj_thresh = self._threshold[0]
        for obj in objmsg.objects.objects:
            if obj.objectness < obj_thresh:
                continue
            if not self._yolo9000_cls:
                score = obj.objectness
                depth_obj = self.check_object_acceptance(obj)
                if depth_obj:
                    yolo_obj_list.append(obj)
                    depth_obj_list.append(depth_obj)
                    depth_obj_map = self.__transform_depth_obj(depth_obj)
                    depth_obj_map_list.append(depth_obj_map)
                    if score >= max_score:
                        max_prob = score
                        max_score = score
                        max_yolo_obj = obj
                        max_depth_obj = depth_obj
                        max_depth_obj_map = depth_obj_map
            else:
                for cls, thresh in zip(self._yolo9000_cls, self._threshold):
                    prob = obj.objectness * \
                        self.tr.probability(obj.class_probability, cls)
                    if self.debug_cv2_image_save_dir is not None or self.debug_image_pub:
                        cv2.rectangle(__rgbimg, (int(obj.left), int(obj.top)), (int(
                            obj.right), int(obj.bottom)), (0, 255, 0), 3)
                        cv2.putText(__rgbimg, 'name={}'.format(cls), (int(obj.left), int(
                            obj.top)), cv2.FONT_HERSHEY_PLAIN, 1., (0, 0, 255))
                        cv2.putText(__rgbimg, 'indices=({},{},{})'.format(obj.row, obj.column, obj.box_index),
                                    (int(obj.left), int(obj.top+20)), cv2.FONT_HERSHEY_PLAIN, 1., (0, 0, 255))
                        cv2.putText(__rgbimg, 'confidence={}'.format(obj.objectness*prob),
                                    (int(obj.left), int(obj.top+40)), cv2.FONT_HERSHEY_PLAIN, 1., (0, 0, 255))
                    #print('cls:{}, prob:{}, thresh:{}'.format(cls, prob, thresh))
                    if prob > max_prob:
                        max_prob = prob
                    score = prob - thresh
                    #print('prob:{}, thresh:{}'.format(prob, thresh))
                    if score < 0:
                        continue
                    #print('before self.check_object_accecptance() call')
                    depth_obj = self.check_object_acceptance(obj)
                    if depth_obj:
                        if self.debug_cv2_image_save_dir is not None or self.debug_image_pub:
                            cv2.rectangle(__rgbimg, (int(obj.left), int(obj.top)), (int(
                                obj.right), int(obj.bottom)), (0, 255, 0), 3)
                            cv2.putText(__rgbimg, 'name={}'.format(cls), (int(obj.left), int(
                                obj.top)), cv2.FONT_HERSHEY_PLAIN, 1., (0, 0, 255))
                            cv2.putText(__rgbimg, 'indices=({},{},{})'.format(obj.row, obj.column, obj.box_index),
                                        (int(obj.left), int(obj.top+20)), cv2.FONT_HERSHEY_PLAIN, 1., (0, 0, 255))
                            cv2.putText(__rgbimg, 'confidence={}'.format(obj.objectness*prob),
                                        (int(obj.left), int(obj.top+40)), cv2.FONT_HERSHEY_PLAIN, 1., (0, 0, 255))
                        #print('depth_obj is not None.')
                        yolo_obj_list.append(obj)
                        depth_obj_list.append(depth_obj)
                        '''
			print('Before::pos.x:{}, pos.y:{}, pos.z:{}, ori.x:{}, ori.y:{}, ori.z:{}, ori.w:{}, frame_id:{}'.format(depth_obj.approachPose.pose.position.x,\
			depth_obj.approachPose.pose.position.y,\
			depth_obj.approachPose.pose.position.z,\
			depth_obj.approachPose.pose.orientation.x,
			depth_obj.approachPose.pose.orientation.y,\
			depth_obj.approachPose.pose.orientation.z + 0.1,\
			depth_obj.approachPose.pose.orientation.w,\
			depth_obj.approachPose.header.frame_id))
			'''
                        depth_obj_map = self.__transform_depth_obj(depth_obj)
                        '''
			print('After::pos.x:{}, pos.y:{}, pos.z:{}, ori.x:{}, ori.y:{}, ori.z:{}, ori.w:{}, frame_id:{}'.format(depth_obj.approachPose.pose.position.x,\
			depth_obj.approachPose.pose.position.y,\
			depth_obj.approachPose.pose.position.z,\
			depth_obj.approachPose.pose.orientation.x,
			depth_obj.approachPose.pose.orientation.y,\
			depth_obj.approachPose.pose.orientation.z + 0.1,\
			depth_obj.approachPose.pose.orientation.w,\
			depth_obj.approachPose.header.frame_id))
			print('Map::pos.x:{}, pos.y:{}, pos.z:{}, ori.x:{}, ori.y:{}, ori.z:{}, ori.w:{}, frame_id:{}'.format(depth_obj_map.approachPose.pose.position.x,\
			depth_obj_map.approachPose.pose.position.y,\
			depth_obj_map.approachPose.pose.position.z,\
			depth_obj_map.approachPose.pose.orientation.x,
			depth_obj_map.approachPose.pose.orientation.y,\
			depth_obj_map.approachPose.pose.orientation.z + 0.1,\
			depth_obj_map.approachPose.pose.orientation.w,\
			depth_obj_map.approachPose.header.frame_id))
			'''
                        depth_obj_map_list.append(depth_obj_map)
                        if score >= max_score:
                            max_score = score
                            max_cls = cls
                            max_yolo_obj = obj
                            max_depth_obj = depth_obj
                            max_depth_obj_map = depth_obj_map
        if max_depth_obj:
            self.found_point_cloud2 = pointmsg
            self.found_yolo_object = max_yolo_obj
            self.found_yolo_objects.extend(yolo_obj_list)
            self.found_depth_object = max_depth_obj
            self.found_depth_objects.extend(depth_obj_list)
            self.found_depth_object_map = max_depth_obj_map
            self.found_depth_objects_map.extend(depth_obj_map_list)
            self.found_class = max_cls
            rospy.loginfo('Max:: class:{}, probability:{}, score:{}'.format(
                max_cls, max_prob, max_score))
            if self.debug_cv2_image_save_dir is not None:
                import datetime
                cv2.imwrite(self.debug_cv2_image_save_dir+'/debug_cv2_find_object_' +
                            datetime.datetime.now().strftime("%s") + '.jpg', __rgbimg)
            if self.debug_image_pub:
                try:
                    imgmsg = self.cv_bridge.cv2_to_imgmsg(__rgbimg)
                    self.debug_image_pub.publish(imgmsg)
                except:
                    rospy.logwarn('Failed to publish debug image.')
        #print('__point_cloud2_cb() end')

    def __transform_depth_obj(self, depth_obj, frame_id_to='map'):
        depth_obj_map = copy.deepcopy(depth_obj)
        depth_obj_map.center = transform_geometry_point_stamped(self.tf_buffer,
                                                                depth_obj.center,
                                                                frame_id_to)
        depth_obj_map.graspPose = transform_geometry_pose_stamped(self.tf_buffer,
                                                                  depth_obj.graspPose,
                                                                  frame_id_to)
        depth_obj_map.approachPose = transform_geometry_pose_stamped(self.tf_buffer,
                                                                     depth_obj.approachPose,
                                                                     frame_id_to)
        return depth_obj_map
