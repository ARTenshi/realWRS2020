#!/usr/bin/env python

import rospy
import smach
import traceback
import numpy as np
from contextlib import nested

from common import speech
from common.smach_states.utils import TemporarySubscriber


class FollowPerson(smach.State):
    def __init__(self, ref_frame='map', timeout=15.,
                 say_fn=None,
                 # FIXME: "car" is not general: specify it when calling this constructor
                 stop_signal=["stop", 'here is',
                              'here is the car', 'car', 'the car'],
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
        self.sphinx_rule = self.stop_signal + \
            ['start', 'follow me', '/300/ <NULL>']
        self.lost_msg = lost_msg
        self.stop_msg = stop_msg
        self.success_msg = success_msg
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.base_cli = actionlib.SimpleActionClient(
            '/move_base/move', MoveBaseAction)
        self.base_cli.wait_for_server(rospy.Duration(1.))
        rospy.wait_for_service('/viewpoint_controller/stop')
        rospy.wait_for_service('/viewpoint_controller/start')
        self.vp_stop = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
        self.vp_start = rospy.ServiceProxy(
            '/viewpoint_controller/start', Empty)
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

    def particle_cb(self, msg):
        from geometry_msgs.msg import PointStamped, PoseStamped
        import tf2_geometry_msgs
        points = np.zeros((len(msg.particles), 2), dtype=np.float32)
        #velocities = np.zeros((len(msg.particles), 2), dtype=np.float32)
        for i, particle in enumerate(msg.particles):
            points[i, :] = particle.position.x, particle.position.y
            #velocities[i,:] = particle.velocity.x, particle.velocity.y
        x, y = points.mean(0)
        var_x, var_y = points.var(0)
        self.target_std = np.sqrt(var_x + var_y)
        #vx, vy = velocities.mean(0)
        ps = PointStamped()
        ps.point.x = x
        ps.point.y = y
        ps.header.frame_id = msg.header.frame_id
        ps_base = self.xtion.tf_buffer.transform(
            ps, 'base_footprint', rospy.Duration(1.))
        x = ps_base.point.x
        y = ps_base.point.y
        d = np.sqrt(x**2+y**2)
        theta = np.arctan2(y, x)
        if d > 5.:
            ps_base.point.x = x - x/d  # arata added 0.5
            ps_base.point.y = y - y/d
            #ps = self.xtion.tf_buffer.transform(ps_base, ps.header.frame_id, rospy.Duration(1.))
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'base_footprint'  # ps.header.frame_id
        self.target_pose.pose.position.x = ps_base.point.x
        self.target_pose.pose.position.y = ps_base.point.y
        self.target_pose.pose.orientation.z = np.sin(theta/2)
        self.target_pose.pose.orientation.w = np.cos(theta/2)

    def sphinx_cb(self, msg):
        for sig in self.stop_signal:
            if sig in msg.data:
                self.stop_requested = True

    def execute(self, userdata):
        try:
            import time
            from move_base_msgs.msg import MoveBaseGoal
            from geometry_msgs.msg import Point, Vector3
            from object_tracker.msg import ParticleArray
            from std_msgs.msg import String
            self.vp_stop()
            self.yolo_reconf.update_configuration(
                {'grid_width': 16, 'grid_height': 12})
            self.tracker_reconf.update_configuration({"fixed_frame": "map",
                                                      "num_particles": 500,
                                                      "position_transition_uncertainty": 0.22,
                                                      "velocity_transition_uncertainty": 0.22,
                                                      "measurement_uncertainty": 0.165,
                                                      "position_measurement_uncertainty": 0.41,
                                                      "surf_distance_allowance": 0.3,
                                                      "surf_min_hessian": 800.00})  # TODO

            self.say_fn(self.wait_msg)
            time.sleep(5.)
            while not rospy.is_shutdown():
                try:
                    rospy.loginfo('Trying to find and memorize the person.')
                    self.memorize_person(add=False, maximum_distance=3.)
                    break
                except:
                    time.sleep(1.)
            rospy.loginfo('Initializing the tracking.')
            self.reset_tracking(mean_position=Point(x=1.4, y=0., z=1.),
                                mean_velocity=Vector3(),
                                covariance=[1, 0, 0, 0, 0, 0,
                                            0, 1, 0, 0, 0, 0,
                                            0, 0, .2, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0],
                                frame_id='base_footprint')
            self.say_fn(self.start_msg)
            self.stop_requested = False
            rate = rospy.Rate(1)
            self.sphinx.set_single_rule(self.sphinx_rule)
            self.target_pose = None
            self.target_std = 0.
            particle_sub = TemporarySubscriber(
                self.particle_topic, ParticleArray, self.particle_cb)
            sphinx_sub = TemporarySubscriber(
                self.sphinx.speech_topic, String, self.sphinx_cb)
            with nested(particle_sub, sphinx_sub, *self.contexts):
                while not rospy.is_shutdown():
                    rospy.loginfo('Following the person.')
                    if self.target_pose is not None:
                        look_pos = self.target_pose.pose.position
                        try:
                            self.xtion.look_at((look_pos.x, look_pos.y, 1.3),
                                               self.target_pose.header.frame_id, move_hand=False)
                        except:
                            rospy.logwarn('look_at() failed.')
                        if not self.stop_requested:
                            self.base_cli.send_goal(
                                MoveBaseGoal(target_pose=self.target_pose))
                    if self.target_std > 10.:
                        self.say_fn(self.lost_msg)
                        self.vp_start()
                        return 'lost'
                    if self.stop_requested:
                        self.base_cli.cancel_goal()
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
