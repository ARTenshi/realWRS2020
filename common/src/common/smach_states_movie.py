#!/usr/bin/env python

import rospy
import smach
import time
import types
from common import speech
import traceback
from depth_lib import gripper_pose
from common import rospose_to_tmcpose
import numpy as np
from contextlib import nested

# Following classes use tf2_ros.Buffer
# NearDoorContext
# FindObject
# HandObject
# PutObject
# FollowPerson
# OpenDoor
# FindPlacement

class TemporarySubscriber:
    def __init__(self, name, msg, cb):
        self.name = name
        self.msg = msg
        self.cb = cb

    def __enter__(self):
        self.sub = rospy.Subscriber(self.name, self.msg, self.cb)
        return self.sub

    def __exit__(self, exctype, excval, traceback):
        self.sub.unregister()

class NearDoorContext(TemporarySubscriber):
    def __init__(self, topic="/laser_2d_pose",
                 door_poses=[],
                 ref_frame='map',
                 near_threshold = 1.5,
                 value_when_near = 0.15,
                 value_when_far = 0.35,
                 tf_buffer=None):
        import dynamic_reconfigure.client as reconf_client
        from geometry_msgs.msg import PoseWithCovarianceStamped
        TemporarySubscriber.__init__(self, topic, PoseWithCovarianceStamped,
                                     self.cb)
        self.obstacle_reconf = reconf_client.Client('obstacle_grid_mapper')
        self.ref_frame = ref_frame
        self.tf_buffer = tf_buffer
        self.door_poses = door_poses
        self.warned = False
        self.near_threshold = near_threshold
        self.value_when_near = value_when_near
        self.value_when_far = value_when_far

    def cb(self, msg):
        if self.ref_frame != msg.header.frame_id:
            if self.tf_buffer is None:
                if not self.warned:
                    rospy.logerr('When the frame of door positions and robot position are different, an instance of tf2_ros.Buffer is required.')
                    self.warned = True
                return
            import tf2_geometry_msgs
            msg = self.tf_buffer.transform(msg, self.ref_frame, rospy.Duration(1.))
        pos = msg.pose.pose.position
        rospy.loginfo("I'm at ({}, {})".format(pos.x, pos.y))
        value = self.value_when_far
        for (x,y), theta in self.door_poses:
            if (x-pos.x)**2+(y-pos.y)**2 < self.near_threshold**2:
                rospy.loginfo("Close to the door at ({}, {})".format(x, y))
                value = self.value_when_near
                break
        self.obstacle_reconf.update_configuration({'exception_potential_distance': value})

class WaitDoorOpen(smach.State):
    def __init__(self, threshold=1.5, timeout=120.,
                 say_fn=None,
                 knock_msg="Knock knock.", success_msg="Thank you.",
                 depth_topic='/hsrb/head_rgbd_sensor/depth_registered/image'):
        from cv_bridge import CvBridge
        smach.State.__init__(self, outcomes=['success', 'timeout', 'failure'])
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.door_is_open = False
        self.current_value = None
        self.threshold = threshold
        self.bridge = CvBridge()
        self.timeout = timeout
        self.knock_msg = knock_msg
        self.success_msg = success_msg
        self.depth_topic = depth_topic

    def execute(self, userdata):
      try:
        from sensor_msgs.msg import Image
        self.door_is_open = False
        self.current_value = None
        if self.say_fn and self.knock_msg:
            self.say_fn(self.knock_msg)

        with TemporarySubscriber(self.depth_topic, Image, self.depth_cb):
            t = 0.
            while not rospy.is_shutdown() and not self.door_is_open:
                time.sleep(1.)
                t += 1.
                if t > self.timeout:
                    break
                rospy.loginfo('Waiting for the door to open. value={}, threshold={}'.format(self.current_value, self.threshold))
        if self.door_is_open:
            if self.say_fn and self.success_msg:
                self.say_fn(self.success_msg)
            return 'success'
        return 'timeout'
      except:
        rospy.logerr(traceback.format_exc())
        return 'failure'

    def depth_cb(self, msg):
        import numpy as np
        img = self.bridge.imgmsg_to_cv2(msg)
        self.current_value = img[np.isfinite(img)].mean()
        if self.current_value > self.threshold:
            self.door_is_open = True

# Waits the gripper to be pushed down.
# If threshold < 0, this state waits the gripper to be released.
class WaitHandPushed(smach.State):
    def __init__(self, threshold=12., timeout=120.,
                 say_fn=None,
                 prompt_msg="Please push down my hand.",
                 success_msg="OK."):
        smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'])
        self.threshold = threshold
        self.pushed = False
        self.current_value = None
        self.timeout = timeout
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.prompt_msg = prompt_msg
        self.success_msg = success_msg

    def execute(self, userdata):
      try:
        from geometry_msgs.msg import WrenchStamped
        self.pushed = False
        self.current_value = None
        if self.say_fn and self.prompt_msg:
            self.say_fn(self.prompt_msg)

        with TemporarySubscriber('/hsrb/wrist_wrench/raw', WrenchStamped, self.wrench_cb):
            t = 0.
            while not rospy.is_shutdown() and not self.pushed:
                time.sleep(1.)
                t += 1.
                if self.timeout is not None and t > self.timeout:
                    break
                rospy.loginfo('Waiting for the hand to be pushed. value={}, threshold={}'.format(self.current_value, self.threshold))
        if self.pushed:
            if self.say_fn and self.success_msg:
                self.say_fn(self.success_msg)
            return 'success'
        return 'timeout'
      except:
        rospy.logerr(traceback.format_exc())
        return 'failure'

    def wrench_cb(self, msg):
        self.current_value = msg.wrench.force.x
        if self.threshold > 0.:
            if self.current_value > self.threshold:
                self.pushed = True
        else:
            if self.current_value < -self.threshold:
                self.pushed = True

class GetCommand(smach.State):
    def __init__(self, listen_fn, parse_fn,
                 use_bypass=True, timeout=30., retry=2,
                 say_fn=None,
                 prompt_msg="Please give me a command.",
                 miss_msg="Sorry. I don't understand.",
                 bypass_msg="Could you give it to me by QR code?",
                 success_msg="I got it.",
                 qr_image_topic='/hsrb/head_rgbd_sensor/rgb/image_rect_mono'):
        from cv_bridge import CvBridge
        import zbar

        smach.State.__init__(self, outcomes=['success', 'bypass', 'failure', 'timeout'],
                             output_keys=['command'])
        self.timeout = timeout
        self.retry = retry
        self.listen_fn = listen_fn
        self.parse_fn = parse_fn
        self.use_bypass = use_bypass
        self.bridge = CvBridge()
        self.scanner = zbar.ImageScanner()
        self.scanner.parse_config('enable')
        self.qr_data = []
        self.prompt_msg = prompt_msg
        self.miss_msg = miss_msg
        self.bypass_msg = bypass_msg
        self.success_msg = success_msg
        self.qr_image_topic = qr_image_topic
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say

    def execute(self, userdata):
      try:
        self.qr_data = []
        from sensor_msgs.msg import Image
        with TemporarySubscriber(self.qr_image_topic, Image, self.image_cb):
            count = 0
            while count <= self.retry:
                if self.use_bypass and self.qr_data:
                    parse = filter(self.parse_fn, self.qr_data)
                    if parse:
                        if self.say_fn and self.success_msg:
                            self.say_fn(self.success_msg)
                        userdata.command = parse[0]
                        return 'success'
                if self.say_fn and self.prompt_msg:
                    self.say_fn(self.prompt_msg)
                try:
                    text = self.listen_fn(self.timeout)
                except:
                    rospy.logerr(traceback.format_exc())
                    if self.preempt_requested():
                        return 'timeout'
                    if not self.use_bypass:
                        return 'timeout'
                    break
                if self.parse_fn(text):
                    if self.say_fn and self.success_msg:
                        self.say_fn(self.success_msg)
                    userdata.command = text
                    return 'success'
                if self.say_fn and self.miss_msg:
                    self.say_fn(self.miss_msg)
                time.sleep(2)
                count += 1

            if not self.use_bypass:
                return 'timeout'

            if self.say_fn and self.bypass_msg:
                self.say_fn(self.bypass_msg)

            elapsed = 0.
            while not rospy.is_shutdown() and (not self.timeout or elapsed < self.timeout):
                parse = filter(self.parse_fn, self.qr_data)
                if parse:
                    if self.say_fn and self.success_msg:
                        self.say_fn(self.success_msg)
                    userdata.command = parse[0]
                    return 'success'
                time.sleep(.2)
                elapsed += .2
        return 'timeout'
      except:
        rospy.logerr(traceback.format_exc())
        return 'failure'

    def image_cb(self, msg):
        import PIL.Image
        import zbar

        img = self.bridge.imgmsg_to_cv2(msg)
        pimg = PIL.Image.fromarray(img)
        zimg = zbar.Image(img.shape[1], img.shape[0],
                          'Y800', pimg.tostring())
        if self.scanner.scan(zimg):
            self.qr_data = [symbol.data for symbol in zimg]

class Navigate(smach.State):
    def __init__(self, goal_pose,
                 ref_frame='map', timeout=120.,
                 say_fn=None,
                 start_msg="",
                 timeout_msg="",
                 success_msg="",
                 contexts=[],
                 spin_fn=None,
                 input_keys=[], additional_outcomes=[]):
        import actionlib
        from move_base_msgs.msg import MoveBaseAction
        smach.State.__init__(self, outcomes=['success', 'failure', 'timeout']+additional_outcomes,
                             input_keys=input_keys)
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

def default_accept_object_fn(userdata, obj):
    depth = obj.center.point.z
    return depth > .3 and depth < 1.5
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
                 get_object_srv_type=None,
                 get_object_srv_kwarg={},
                 get_object_trials = 3,
                 accept_object_fn=default_accept_object_fn,
                 names_srv='get_names',
                 timeout=30.,
                 say_fn=None,
                 start_msg="Looking for an object.",
                 timeout_msg="",
                 success_msg="Found an object.",
                 camera_joints=[{'lift':0., 'tilt':0., 'pan':0.},
                                {'lift':0., 'tilt':0., 'pan':-.8},
                                {'lift':0., 'tilt':0., 'pan':.8}],
                 tf_buffer=None):
        from common import HSRB_Xtion
        smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
                             input_keys=input_keys,
                             output_keys=['object', 'yolo_object'])
        self.yolo9000_cls = yolo9000_cls
        self.threshold = threshold
        self.obj_topic = obj_topic
        self.timeout = timeout
        self.start_msg = start_msg
        self.timeout_msg = timeout_msg
        self.success_msg = success_msg
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.found_depth_object = None
        self.found_yolo_object = None
        self.found_class = None
        from yolo_tf.libs import TreeReader
        self.tr = TreeReader(names_srv)
        if get_object_srv_type is None:
            from depth_lib.srv import GetObject
            get_object_srv_type = GetObject
        rospy.wait_for_service(get_object_srv)
        self.get_object = rospy.ServiceProxy(get_object_srv,
                                             get_object_srv_type)
        self.get_object_srv_kwarg = get_object_srv_kwarg
        self.get_object_trials = get_object_trials
        if accept_object_fn:
            self.accept_object_fn = accept_object_fn
        else:
            self.accept_object_fn = lambda ud, obj: True
        self.xtion = HSRB_Xtion(tf_buffer)
        self.camera_joints = camera_joints

    def execute(self, userdata):
      try:
        import math
        from yolo_tf.msg import ObjectArray

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

        if self.say_fn and self.start_msg:
            try:
                msg = self.start_msg.format(**userdata)
                self.say_fn(msg)
            except:
                rospy.logerr(traceback.format_exc())

        userdata.object = None
        userdata.yolo_object = None
        self.found_depth_object = None
        self.found_yolo_object = None
        self._userdata = userdata
        joint_ind = 0
        joint_count = 0
        with TemporarySubscriber(self.obj_topic, ObjectArray, self.obj_cb):
            t = 0.
            while not rospy.is_shutdown() and not self.found_depth_object:
                if self.camera_joints and joint_count % 50 == 0:
                    print self.camera_joints[joint_ind]
                    self.xtion.move(wait=True, **self.camera_joints[joint_ind])
                    joint_ind = (joint_ind+1) % len(self.camera_joints)
                joint_count += 1
                time.sleep(.1)
                t += .1
                if t > self.timeout:
                    if self.say_fn and self.timeout_msg:
                        try:
                            msg = self.timeout_msg.format(**userdata)
                            self.say_fn(msg)
                        except:
                            rospy.logerr(traceback.format_exc())
                    return 'timeout'

        userdata.object = self.found_depth_object
        userdata.yolo_object = self.found_yolo_object
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
                                            **self._get_object_srv_kwarg).object
            except:
                rospy.logerr(traceback.format_exc())
                continue
            if self.accept_object_fn(self._userdata, depth_obj):
                rospy.loginfo('Object rejected by accept_object_fn.')
                break
            else:
                depth_obj = None
        return depth_obj

    def obj_cb(self, msg):
        max_prob = 0
        max_score = -1
        max_cls = -1
        max_yolo_obj = None
        max_depth_obj = None
        obj_thresh = 0.1
        if not self._yolo9000_cls and self._threshold:
            obj_thresh = self._threshold[0]
        for obj in msg.objects:
            if obj.objectness < obj_thresh:
                continue
            if not self._yolo9000_cls:
                score = obj.objectness
                if score < max_score:
                    continue
                depth_obj = self.check_object_acceptance(obj)
                if depth_obj:
                    max_prob = score
                    max_score = score
                    max_yolo_obj = obj
                    max_depth_obj = depth_obj
            else:
              for cls, thresh in zip(self._yolo9000_cls, self._threshold):
                prob = obj.objectness*self.tr.probability(obj.class_probability, cls)
                if prob > max_prob:
                    max_prob = prob
                score = prob - thresh
                if score < 0 or score < max_score:
                    continue
                depth_obj = self.check_object_acceptance(obj)
                if depth_obj:
                    max_score = score
                    max_cls = cls
                    max_yolo_obj = obj
                    max_depth_obj = depth_obj
        if max_depth_obj:
            self.found_yolo_object = max_yolo_obj
            self.found_depth_object = max_depth_obj
            self.found_class = max_cls
        rospy.loginfo('Max probability: {}'.format(max_prob))

class GraspObject(smach.State):
    def __init__(self, robot,
                 say_fn=None,
                 start_msg="",
                 success_msg=""):
        smach.State.__init__(self, outcomes=['success', 'failure'],
                             input_keys=['object'])
        self.start_msg = start_msg
        self.success_msg = success_msg
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.whole_body = robot.get('whole_body')
        self.gripper = robot.get('gripper')

    def execute(self, userdata):
      try:
        if self.say_fn and self.start_msg:
            self.say_fn(self.start_msg)

        from hsrb_interface import geometry
        pose = userdata.object.approachPose.pose
        pos = geometry.Vector3(x = pose.position.x,
                               y = pose.position.y,
                               z = pose.position.z)
        ori = geometry.Quaternion(x = pose.orientation.x,
                                  y = pose.orientation.y,
                                  z = pose.orientation.z,
                                  w = pose.orientation.w)
        frame_id = userdata.object.approachPose.header.frame_id
        self.whole_body.move_end_effector_pose((pos, ori), frame_id)
        pose = userdata.object.graspPose.pose
        pos = geometry.Vector3(x = pose.position.x,
                               y = pose.position.y,
                               z = pose.position.z)
        ori = geometry.Quaternion(x = pose.orientation.x,
                                  y = pose.orientation.y,
                                  z = pose.orientation.z,
                                  w = pose.orientation.w)
        frame_id = userdata.object.graspPose.header.frame_id
        self.gripper.command(1.)
        self.whole_body.move_end_effector_pose((pos, ori), frame_id)
        self.gripper.grasp(-0.01)
        pos = geometry.Vector3(-pos.x, -pos.y, -pos.z)
        self.whole_body.move_end_effector_pose((pos, ori), frame_id)

        if self.say_fn and self.success_msg:
            self.say_fn(self.success_msg)
        return 'success'
      except:
        rospy.logerr(traceback.format_exc())
        return 'failure'

class HandObject(smach.State):
    def __init__(self, robot,
                 threshold=10.,
                 timeout=10.,
                 say_fn=None,
                 start_msg="Here you are.",
                 success_msg="",
                 tf_buffer=None):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'failure'],
                             input_keys=['person'])
        self.start_msg = start_msg
        self.success_msg = success_msg
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.whole_body = robot.get('whole_body')
        self.gripper = robot.get('gripper')
        self.timeout = timeout
        self.threshold = threshold
        self.pulled = False
        self.current_value = None
        self.initial_force = None
        import tf2_ros
        import tf2_geometry_msgs
        if tf_buffer is None:
            self.tf_buf = tf2_ros.Buffer(rospy.Duration(5.))
            tf2_ros.TransformListener(self.tf_buf)
        else:
            self.tf_buf = tf_buffer

    def execute(self, userdata):
      try:
        import math
        from geometry_msgs.msg import WrenchStamped
        self.pulled = False
        self.current_value = None

        from hsrb_interface import geometry
        frame_id = 'base_footprint'
        point = self.tf_buf.transform(userdata.person.center, frame_id,
                                      rospy.Duration(1.))
        dx = point.point.x
        dy = point.point.y
        d = math.sqrt(dx**2 + dy**2)
        dx /= d
        dy /= d
        pos = geometry.Vector3(x = point.point.x - dx*.3,
                               y = point.point.y - dy*.3,
                               z = point.point.z)
        ori = self.whole_body.get_end_effector_pose()[1]
        self.whole_body.move_end_effector_pose((pos, ori), frame_id)

        self.initial_force = rospy.wait_for_message('/hsrb/wrist_wrench/raw', WrenchStamped).wrench.force

        if self.say_fn and self.start_msg:
            self.say_fn(self.start_msg)

        with TemporarySubscriber('/hsrb/wrist_wrench/raw', WrenchStamped, self.wrench_cb):
            t = 0.
            while not rospy.is_shutdown() and not self.pulled:
                time.sleep(.1)
                t += .1
                if t > self.timeout:
                    break
                rospy.loginfo('Waiting for the person to pull the object. value={}, threshold={}'.format(self.current_value, self.threshold))
        if self.pulled:
            if self.say_fn and self.success_msg:
                self.say_fn(self.success_msg)
            self.gripper.grasp(0.01)
            return 'success'
        return 'timeout'
      except:
        rospy.logerr(traceback.format_exc())
        return 'failure'

    def wrench_cb(self, msg):
        import math
        f = msg.wrench.force
        d = (f.x-self.initial_force.x)**2 + (f.y-self.initial_force.y)**2 + (f.z-self.initial_force.z)**2
        self.current_value = math.sqrt(d)
        if self.current_value > self.threshold:
            self.pulled = True

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
        poses = gripper_pose((pos.point.x, pos.point.y, pos.point.z),
                             (0,-1,0), (1,0,0), back_dist=.3)
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

class AskName(smach.State):
    def __init__(self, name_candidates, say_fn=None,
                 ask_msg="Excuse me. Could I have your name?",
                 ask_again_msg="Could you tell me your name again?",
                 confirm_msg="Your name is {person_name}?",
                 confirm_again_msg="Please say yes or no.",
                 success_msg="Thank you.",
                 timeout=30.):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'failure'],
                             output_keys=['person_name'])
        self.ask_msg = ask_msg
        self.confirm_msg = confirm_msg
        self.ask_again_msg = ask_again_msg
        self.confirm_again_msg = confirm_again_msg
        self.success_msg = success_msg
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.name_candidates = map(lambda x: x.lower(), name_candidates)
        self.timeout = timeout
        from pocketsphinx_jsgf import PocketSphinxClient
        self.sphinx = PocketSphinxClient()

    def execute(self, userdata):
      try:
        msg_dict = dict(userdata)
        self.say_fn(self.ask_msg.format(**msg_dict))
        history = {}
        while not rospy.is_shutdown():
            self.sphinx.set_single_rule(self.name_candidates)
            rospy.loginfo('Name candidates: {}'.format(self.name_candidates))
            try:
                name = self.sphinx.next_speech(timeout=timeout)
            except:
                return 'timeout'
            rospy.loginfo('Name: {}'.format(name))
            if name in self.name_candidates:
                if name in history:
                    history[name] += 1
                else:
                    history[name] = 1
                userdata.person_name = name
                msg_dict['person_name'] = name
                for k,v in history.items():
                    if v>=3:
                        return 'success'
                self.say_fn(self.confirm_msg.format(msg_dict))
                try:
                    self.sphinx.next_yes_or_no(timeout=timeout)
                except:
                    return 'timeout'
                if confirm not in ['yes', 'no']:
                    self.say_fn(self.confirm_again_msg.format(msg_dict))
                    try:
                        self.sphinx.next_yes_or_no(timeout=timeout)
                    except:
                        return 'timeout'
                if confirm == 'yes':
                    self.say_fn(self.success_msg.format(msg_dict))
                    return 'success'
            self.say_fn(self.ask_again_msg.format(msg_dict))
        rospy.logerr("Failed to get the person's name.")
        return 'failure'
      except:
        rospy.logerr(traceback.format_exc())
        return 'failure'

class FollowPerson(smach.State):
    def __init__(self, ref_frame='map', timeout=15.,
                 say_fn=None,
                 # FIXME: "car" is not general: specify it when calling this constructor
                 stop_signal=["stop", 'here is', 'here is the car', 'car', 'the car'],
                 wait_msg="Please stand in front of me, with your back to me.",
                 start_msg="I'm following you until you say stop.",
                 lost_msg="I've lost you.",
                 stop_msg="I stopped. Is this the destination?",
                 success_msg="Thank you for guiding me.",
                 particle_topic='person_particles',
                 reset_srv='reset_person_tracking',
                 memorize_srv='memorize_person',
                 contexts=[],
                 tf_buffer=None):
        import actionlib
        from move_base_msgs.msg import MoveBaseAction
        from common import HSRB_Xtion
        from object_tracker.srv import MemorizePerson, ResetTracking
        from std_msgs.msg import String
        from std_srvs.srv import Empty
        from pocketsphinx_jsgf import PocketSphinxClient
        import dynamic_reconfigure.client as reconf_client
        import threading
        from geometry_msgs.msg import Twist, Pose
        smach.State.__init__(self, outcomes=['stop_request', 'failure', 'lost'],
                             input_keys=['object'])
        self.ref_frame = ref_frame
        self.timeout = timeout
        self.wait_msg = wait_msg
        self.start_msg = start_msg
        if isinstance(stop_signal, str):
            self.stop_signal = [stop_signal] if stop_signal else []
        elif stop_signal is None:
            self.stop_signal = []
        else:
            self.stop_signal = stop_signal
        self.sphinx_rule = self.stop_signal + ['start', 'follow me', '/300/ <NULL>']
        self.lost_msg = lost_msg
        self.stop_msg = stop_msg
        self.success_msg = success_msg
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.base_cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.base_cli.wait_for_server(rospy.Duration(1.))
        rospy.wait_for_service('/viewpoint_controller/stop')
        rospy.wait_for_service('/viewpoint_controller/start')
        self.vp_stop = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
        self.vp_start = rospy.ServiceProxy('/viewpoint_controller/start', Empty)
        self.xtion = HSRB_Xtion(tf_buffer)
        self.particle_topic = particle_topic
        self.sphinx = PocketSphinxClient()
        rospy.wait_for_service(reset_srv)
        self.reset_tracking = rospy.ServiceProxy(reset_srv, ResetTracking)
        rospy.wait_for_service(memorize_srv)
        self.memorize_person = rospy.ServiceProxy(memorize_srv, MemorizePerson)
        self.stop_requested = False
        self.yolo_reconf = reconf_client.Client('yolo_detector')
        self.tracker_reconf = reconf_client.Client('person_tracker')
        self.contexts = contexts

        #####################################

        self.cmd_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist)

        #####################################

    def particle_cb(self, msg):
        from geometry_msgs.msg import PointStamped, PoseStamped
        import tf2_geometry_msgs
        points = np.zeros((len(msg.particles), 2), dtype=np.float32)
        #velocities = np.zeros((len(msg.particles), 2), dtype=np.float32)
        for i, particle in enumerate(msg.particles):
            points[i,:] = particle.position.x, particle.position.y
            #velocities[i,:] = particle.velocity.x, particle.velocity.y
        x,y = points.mean(0)
        var_x, var_y = points.var(0)
        self.target_std = np.sqrt(var_x + var_y)
        #vx, vy = velocities.mean(0)
        ps = PointStamped()
        ps.point.x = x
        ps.point.y = y
        ps.header.frame_id = msg.header.frame_id
        ps_base = self.xtion.tf_buffer.transform(ps, 'base_footprint', rospy.Duration(1.))
        x = ps_base.point.x
        y = ps_base.point.y
        d = np.sqrt(x**2+y**2)
        theta = np.arctan2(y, x)
        if d > 5.:
            ps_base.point.x = x - x/d # arata added 0.5
            ps_base.point.y = y - y/d
            #ps = self.xtion.tf_buffer.transform(ps_base, ps.header.frame_id, rospy.Duration(1.))
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'base_footprint'#ps.header.frame_id
        self.target_pose.pose.position.x = ps_base.point.x
        self.target_pose.pose.position.y = ps_base.point.y
        self.target_pose.pose.orientation.z = np.sin(theta/2)
        self.target_pose.pose.orientation.w = np.cos(theta/2)

    def sphinx_cb(self, msg):
        for sig in self.stop_signal:
            if sig in msg.data:
                self.stop_requested = True
    ##################################
    def ichikawa_follow_cb(self, msg):
        from geometry_msgs.msg import Twist
        move_cmd = Twist()
        x = msg.position.x
        z = msg.position.z

        move_cmd.linear.x = -0.7+z
        move_cmd.angular.z = -x*2

        self.cmd_vel_pub.publish(move_cmd)
    #################################

    def execute(self, userdata):
      try:
        import time
        from move_base_msgs.msg import MoveBaseGoal
        from geometry_msgs.msg import Point, Vector3
        from object_tracker.msg import ParticleArray
        from std_msgs.msg import String
        ####
        from geometry_msgs.msg import Pose
        ###
        self.vp_stop()
        self.yolo_reconf.update_configuration({'grid_width': 16, 'grid_height': 12})
        self.tracker_reconf.update_configuration({"fixed_frame":"map",
                                                  "num_particles":500,
                                                  "position_transition_uncertainty":0.22,
                                                  "velocity_transition_uncertainty":0.22,
                                                  "measurement_uncertainty":0.165,
                                                  "position_measurement_uncertainty":0.41,
                                                  "surf_distance_allowance":0.3,
                                                  "surf_min_hessian":800.00}) #TODO

        self.say_fn(self.wait_msg)
        time.sleep(5.)
        while not rospy.is_shutdown():
            try:
                rospy.loginfo('Trying to find and memorize the person.')
                self.memorize_person(add=False, maximum_distance = 3.)
                break
            except:
                time.sleep(1.)
        rospy.loginfo('Initializing the tracking.')
        self.reset_tracking(mean_position = Point(x=1.4, y=0., z=1.),
                            mean_velocity = Vector3(),
                            covariance = [1, 0, 0, 0, 0, 0,
                                          0, 1, 0, 0, 0, 0,
                                          0, 0, .2,0, 0, 0,
                                          0, 0, 0, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0],
                            frame_id = 'base_footprint')
        self.say_fn(self.start_msg)
        self.stop_requested = False
        rate = rospy.Rate(1)
        self.sphinx.set_single_rule(self.sphinx_rule)
        self.target_pose = None
        self.target_std = 0.
        #particle_sub = TemporarySubscriber(self.particle_topic, ParticleArray, self.particle_cb)
        sphinx_sub = TemporarySubscriber(self.sphinx.speech_topic, String, self.sphinx_cb)
        #with nested(particle_sub, sphinx_sub, *self.contexts):
        with nested(sphinx_sub, *self.contexts):
            self.follow_subscriber = rospy.Subscriber('/hsrb/ichikawa/follow/position/cpp', Pose,
                                                       self.ichikawa_follow_cb, queue_size=1)
            while not rospy.is_shutdown():
                rospy.loginfo('Following the person.')
                '''
                if self.target_pose is not None:
                    look_pos = self.target_pose.pose.position
                    try:
                      self.xtion.look_at((look_pos.x, look_pos.y, 1.3),
                                         self.target_pose.header.frame_id, move_hand=False)
                    except:
                      rospy.logwarn('look_at() failed.')
                    if not self.stop_requested:
                        self.base_cli.send_goal(MoveBaseGoal(target_pose = self.target_pose))
                if self.target_std > 10.:
                    self.say_fn(self.lost_msg)
                    self.vp_start()
                    return 'lost'
                '''
                if self.stop_requested:
                    #self.base_cli.cancel_goal()
                    self.follow_subscriber.unregister()
                    self.say_fn(self.stop_msg)
                    try:
                        if self.sphinx.next_yes_or_no(timeout=self.timeout) == 'no':
                            self.say_fn(self.start_msg)
                            self.stop_requested = False
                            self.sphinx.set_single_rule(self.sphinx_rule)
                            continue
                    except:
                        pass
                    self.vp_start()
                    self.say_fn(self.success_msg)
                    return 'stop_request'
                rate.sleep()
      except:
        rospy.logerr(traceback.format_exc())
        return 'failure'

class OpenDoor(smach.State):
  def __init__(self, robot, handle_position, timeout=None,
               tf_buffer=None):
    assert handle_position in ['left', 'right']
    from depth_lib.srv import GetParallelPlanes
    import tf2_geometry_msgs
    from common import HSRB_Xtion, HSRB_Impedance
    smach.State.__init__(self, outcomes=['success', 'door_not_found', 'failure'])
    self.timeout = timeout

    rospy.wait_for_service('get_parallel_planes')
    self.get_parallel_planes = rospy.ServiceProxy('get_parallel_planes', GetParallelPlanes)
    self.xtion = HSRB_Xtion(tf_buffer=tf_buffer)
    self.impedance = HSRB_Impedance()
    self.tf_buffer = self.xtion.tf_buffer
    self.whole_body = robot.get('whole_body')
    self.gripper = robot.get('gripper')
    self.omni_base = robot.get('omni_base')
    self.handle_position = handle_position

  def execute(self, userdata):
    try:
      from depth_lib import read_point_cloud, closest_region
      from sensor_msgs.msg import PointCloud2
      from geometry_msgs.msg import Pose, Vector3Stamped, PointStamped
      import cv2

      #self.xtion.move(lift=best_shelf_height+0.4-1.0, wait=True, timeout=self.timeout)
      #self.xtion.look_at((.9, 0, best_shelf_height), 'base_footprint', wait=True, timeout=self.timeout)
      now = rospy.Time.now()
      count = 0
      while not rospy.is_shutdown():
        try:
          pointmsg = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/points', PointCloud2, self.timeout)
          dt = now - pointmsg.header.stamp
          if dt.to_sec() < 0:
            break
          rospy.logwarn('Data is too old: {}'.format(dt.to_sec()))
        except:
          count += 1
          if count > 5:
            return 'failure'
      points, rgbimg = read_point_cloud(pointmsg)
      floor_norm = Vector3Stamped()
      floor_norm.header.frame_id = 'base_footprint'
      floor_norm.vector.z = 1.
      planes = self.get_parallel_planes(floor_norm, 3000, 30, 0.02, .6).planes
      planes = filter(lambda p: p.center.point.z < 1.5, planes)
      planes = filter(lambda p: p.normal.vector.z < -0.5, planes)
      planes = filter(lambda p: abs(p.center.point.x) < 1., planes)
      rospy.loginfo('Found {} door candidates.'.format(len(planes)))
      planes.sort(key=lambda p: p.center.point.z)
      if not planes:
          return 'door_not_found'
      door = planes[0]
      door_center = np.array([door.center.point.x,
                              door.center.point.y,
                              door.center.point.z])
      door_norm = np.array([door.normal.vector.x,
                            door.normal.vector.y,
                            door.normal.vector.z])

      floor_norm = self.tf_buffer.transform(floor_norm, pointmsg.header.frame_id, rospy.Duration(1.)).vector
      camera_pos = PointStamped()
      camera_pos.header.frame_id = pointmsg.header.frame_id
      camera_pos = self.tf_buffer.transform(camera_pos, 'base_footprint', rospy.Duration(1.)).point
      heights = points[:,:,0] * floor_norm.x + points[:,:,1] * floor_norm.y + points[:,:,2] * floor_norm.z
      heights += camera_pos.z
      heights[np.isnan(heights)] = 0.
      door_depth = np.dot(door_norm, door_center)
      depths = points[:,:,0] * door_norm[0] + points[:,:,1] * door_norm[1] + points[:,:,2] * door_norm[2]
      on_handle = np.logical_and(depths > door_depth+0.03, depths < door_depth+0.1)
      on_handle = np.logical_and(on_handle, heights < 1.1)
      on_handle = np.logical_and(on_handle, heights > .8)
      #rgbimg[np.where(on_handle)] = (255, 0, 0)
      #cv2.imshow('Door handle', rgbimg)
      #cv2.waitKey()
      on_handle_uint8 = np.uint8(on_handle*255)
      _, labels = cv2.distanceTransformWithLabels(255-on_handle_uint8, cv2.cv.CV_DIST_L2, 3)
      labels = labels & on_handle_uint8
      if labels.max() == 0:
          return 'failure'
      best_i = 0
      best_width = 0
      for i in xrange(labels.max()):
          inliers = points[np.where(labels==i+1)]
          xs = inliers[:,0]
          xs = xs[np.isfinite(xs)]
          width = xs.max() - xs.min()
          center = (xs.min() + xs.max())/2.
          if (self.handle_position == 'left' and center > door_center[0])\
             or (self.handle_position == 'right' and center < door_center[0]):
              continue
          #print 'Region {}: width={}'.format(i+1, width)
          if width < 0.1 or width > 0.3:
              continue
          if width > best_width:
              best_width = width
              best_i = i+1
      if best_i == 0:
          return 'failure'
      on_handle = labels == best_i
      u, v = map(np.array, np.where(on_handle))
      best_u = u.min()
      if self.handle_position == 'left':
          best_v = int(v.max()*0.7 + v.min()*0.3)
      else:
          best_v = int(v.min()*0.7 + v.max()*0.3)
      best_uv = (best_u, best_v)
      #rgbimg[np.where(on_handle)] = (255, 0, 0)
      #rgbimg[best_uv] = (0, 0, 255)
      #cv2.imshow('Door handle', rgbimg)
      #cv2.waitKey()
      handle_depth = depths[np.logical_and(on_handle, np.isfinite(depths))].mean()
      best_xyz = points[best_uv]
      #best_xyz += (door_depth+0.02 - np.dot(best_xyz, door_norm)) * door_norm
      best_xyz += (handle_depth+0.02 - np.dot(best_xyz, door_norm)) * door_norm
      best_xyz += np.array([floor_norm.x, floor_norm.y, floor_norm.z]) * 0.1
      pose1, _ = gripper_pose(best_xyz, (1,0,0), -door_norm, back_dist=.0)
      pose2 = Pose()
      pose2.position.x = -0.15
      pose2.orientation.w = 1.
      pose3 = Pose()
      pose3.position.x = -0.1
      pose3.position.y = 0.25 if self.handle_position=='left' else -0.25
      pose3.position.z = 0.25
      pose3.orientation.w = 1.
      pose4 = Pose()
      pose4.position.z = 1.
      pose4.orientation.w = 1.
      pose1, pose2, pose3, pose4 = map(rospose_to_tmcpose, [pose1, pose2, pose3, pose4])
      self.gripper.grasp(-0.01)
      self.whole_body.move_end_effector_pose(pose1, pointmsg.header.frame_id)
      self.whole_body.move_end_effector_pose(pose2, 'hand_palm_link')
      self.whole_body.move_end_effector_pose(pose3, 'hand_palm_link')
      self.whole_body.move_end_effector_pose(pose4, 'hand_palm_link')
      self.impedance.move_to_go()
      return 'success'
    except:
      rospy.logerr(traceback.format_exc())
      return 'failure'

class FindPlacement(smach.State):
  def __init__(self, tf_buffer=None, timeout=None):
    #from depth_lib.srv import GetPerpendicularPlanes, GetParallelPlanes
    import tf2_ros
    import tf2_geometry_msgs
    from common import HSRB_Xtion
    smach.State.__init__(self, outcomes=['success', 'failure'],
                         input_keys = ['placement_hint', 'shelf_heights'],
                         output_keys = ['placement'])
    self.timeout = timeout

    #rospy.wait_for_service('get_perpendicular_planes')
    #rospy.wait_for_service('get_parallel_planes')
    #self.get_perpendicular_planes = rospy.ServiceProxy('get_perpendicular_planes', GetPerpendicularPlanes)
    #self.get_parallel_planes = rospy.ServiceProxy('get_parallel_planes', GetParallelPlanes)
    if tf_buffer is None:
        tf_buffer = tf2_ros.Buffer(rospy.Duration(20.))
        tf2_ros.TransformListener(tf_buffer)
    self.tf_buffer = tf_buffer
    self.xtion = HSRB_Xtion(tf_buffer=tf_buffer)

  def execute(self, userdata):
    try:
      from depth_lib import read_point_cloud
      from sensor_msgs.msg import PointCloud2
      from geometry_msgs.msg import Vector3Stamped, PointStamped
      import cv2
      target_height = userdata.placement_hint.point.z

      shelf_heights = sorted(userdata.shelf_heights)
      best_shelf_height = shelf_heights[0]
      upper_shelf_height = shelf_heights[1] if len(shelf_heights) > 1 else shelf_heights[0]+.5
      for i in xrange(len(shelf_heights)):
        h1 = shelf_heights[i]
        h2 = shelf_heights[i+1] if i+1 < len(shelf_heights) else h1+.5
        if target_height >= h1 and target_height < h2:
          best_shelf_height = h1
          upper_shelf_height = h2
          break

      self.xtion.move(lift=best_shelf_height+0.4-1.0, wait=True, timeout=self.timeout)
      self.xtion.look_at((.9, 0, best_shelf_height), 'base_footprint', wait=True, timeout=self.timeout)
      import time
      time.sleep(2.)
      now = rospy.Time.now()
      count = 0
      while not rospy.is_shutdown():
        try:
          pointmsg = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/points', PointCloud2, self.timeout)
          dt = now - pointmsg.header.stamp
          if dt.to_sec() < 0:
            break
          rospy.logwarn('Data is too old: {}'.format(dt.to_sec()))
        except:
          rospy.logerr(traceback.format_exc())
          count += 1
          if count > 5:
            return 'failure'
      points, rgbimg = read_point_cloud(pointmsg)
      floor_norm = Vector3Stamped()
      floor_norm.header.frame_id = 'base_footprint'
      floor_norm.vector.z = 1.
      floor_norm = self.tf_buffer.transform(floor_norm, pointmsg.header.frame_id, rospy.Duration(1.)).vector
      camera_pos = PointStamped()
      camera_pos.header.frame_id = pointmsg.header.frame_id
      camera_pos = self.tf_buffer.transform(camera_pos, 'base_footprint', rospy.Duration(1.)).point
      print 'Best shelf: {}'.format(best_shelf_height)
      heights = points[:,:,0] * floor_norm.x + points[:,:,1] * floor_norm.y + points[:,:,2] * floor_norm.z
      heights += camera_pos.z
      heights[np.isnan(heights)] = 0.
      space = np.logical_and(heights > best_shelf_height-0.06, heights < best_shelf_height+0.03)
      #rgbimg[np.where(space)] = (255, 0, 0)
      #cv2.imshow('Shelf', rgbimg)
      #cv2.waitKey()
      KERNEL_WIDTH = 120
      KERNEL_HEIGHT = 60
      kernel = np.ones((KERNEL_HEIGHT,KERNEL_WIDTH),np.float32)/(KERNEL_WIDTH*KERNEL_HEIGHT)
      #TODO: morphology transform
      space = cv2.filter2D(np.uint8(space)*255,-1,kernel) > 250
      target_cam = self.tf_buffer.transform(userdata.placement_hint, pointmsg.header.frame_id, rospy.Duration(1.)).point
      target_cam = np.array([[[target_cam.x, target_cam.y, target_cam.z]]])
      distances = np.sqrt(((points - target_cam)**2).sum(2))
      score = 1./distances
      score[np.logical_not(space)] = 0.
      score[np.isnan(score)] = 0.
      best_uv = np.unravel_index(score.argmax(), score.shape)

      print 'Placement height: {}'.format(heights[best_uv])
      print 'upper shelf height: {}'.format(upper_shelf_height)
      rgbimg[np.where(space)] = (255, 0, 0)
      rgbimg[best_uv] = (0,0,255)
      #cv2.imshow('Shelf', rgbimg)
      #cv2.waitKey()
      if space.sum() == 0:
        return 'failure'
      placement = PointStamped()
      placement.header.frame_id = pointmsg.header.frame_id
      placement.point.x, placement.point.y, placement.point.z = points[best_uv]
      placement = self.tf_buffer.transform(placement, 'base_footprint', rospy.Duration(1.))
      placement.point.z += min(.1, (upper_shelf_height-best_shelf_height)/2)
      #print placement
      userdata.placement = placement
      return 'success'
    except:
      rospy.logerr(traceback.format_exc())
      return 'failure'

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
        smach.StateMachine.add('TEST', state, transitions={k:k for k in outcomes})
    print 'outcome: '+sm.execute()
    print 'userdata: {}'.format(dict(sm.userdata))
