#!/usr/bin/env python

import rospy
import smach
import traceback
import numpy as np

from depth_lib import gripper_pose

from common import rospose_to_tmcpose


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
        if rospy.get_param('is_sim', False) == False:
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
                    pointmsg = rospy.wait_for_message(
                        '/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, self.timeout)
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
            planes = self.get_parallel_planes(
                floor_norm, 3000, 30, 0.02, .6).planes
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

            floor_norm = self.tf_buffer.transform(
                floor_norm, pointmsg.header.frame_id, rospy.Duration(1.)).vector
            camera_pos = PointStamped()
            camera_pos.header.frame_id = pointmsg.header.frame_id
            camera_pos = self.tf_buffer.transform(
                camera_pos, 'base_footprint', rospy.Duration(1.)).point
            heights = points[:, :, 0] * floor_norm.x + points[:, :, 1] * floor_norm.y + points[:, :, 2] * floor_norm.z
            heights += camera_pos.z
            heights[np.isnan(heights)] = 0.
            door_depth = np.dot(door_norm, door_center)
            depths = points[:, :, 0] * door_norm[0] + points[:, :, 1] * door_norm[1] + points[:, :, 2] * door_norm[2]
            on_handle = np.logical_and(
                depths > door_depth+0.03, depths < door_depth+0.1)
            on_handle = np.logical_and(on_handle, heights < 1.1)
            on_handle = np.logical_and(on_handle, heights > .8)
            #rgbimg[np.where(on_handle)] = (255, 0, 0)
            #cv2.imshow('Door handle', rgbimg)
            # cv2.waitKey()
            on_handle_uint8 = np.uint8(on_handle*255)
            _, labels = cv2.distanceTransformWithLabels(255-on_handle_uint8, cv2.DIST_L2, 3)
            labels = labels & on_handle_uint8
            if labels.max() == 0:
                return 'failure'
            best_i = 0
            best_width = 0
            for i in xrange(labels.max()):
                inliers = points[np.where(labels == i+1)]
                xs = inliers[:, 0]
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
            # cv2.waitKey()
            handle_depth = depths[np.logical_and(
                on_handle, np.isfinite(depths))].mean()
            best_xyz = points[best_uv]
            #best_xyz += (door_depth+0.02 - np.dot(best_xyz, door_norm)) * door_norm
            best_xyz += (handle_depth+0.02 - np.dot(best_xyz, door_norm)) * door_norm
            best_xyz += np.array([floor_norm.x, floor_norm.y, floor_norm.z]) * 0.1
            pose1, _ = gripper_pose(best_xyz, (1, 0, 0), -door_norm, back_dist=.0)
            pose2 = Pose()
            pose2.position.x = -0.15
            pose2.orientation.w = 1.
            pose3 = Pose()
            pose3.position.x = -0.1
            pose3.position.y = 0.25 if self.handle_position == 'left' else -0.25
            pose3.position.z = 0.25
            pose3.orientation.w = 1.
            pose4 = Pose()
            pose4.position.z = 1.
            pose4.orientation.w = 1.
            pose1, pose2, pose3, pose4 = map(
                rospose_to_tmcpose, [pose1, pose2, pose3, pose4])
            self.gripper.grasp(-0.01)
            self.whole_body.move_end_effector_pose(
                pose1, pointmsg.header.frame_id)
            self.whole_body.move_end_effector_pose(pose2, 'hand_palm_link')
            self.whole_body.move_end_effector_pose(pose3, 'hand_palm_link')
            self.whole_body.move_end_effector_pose(pose4, 'hand_palm_link')
            self.impedance.move_to_go()
            print "end try"
            return 'success'
        except:
            rospy.logerr(traceback.format_exc())
            return 'failure'


if __name__ == '__main__':
    def move_gripper_fn(pose):
        from hsrb_interface import geometry
        position = geometry.Vector3(x=pose.pose.position.x,
                                    y=pose.pose.position.y,
                                    z=pose.pose.position.z)
        orientation = geometry.Quaternion(x=pose.pose.orientation.x,
                                          y=pose.pose.orientation.y,
                                          z=pose.pose.orientation.z,
                                          w=pose.pose.orientation.w)
        whole_body.move_end_effector_pose((position, orientation),
                                          pose.header.frame_id)

    def grasp_fn():
        gripper.grasp(-.01)

    import sys
    import re
    import hsrb_interface

    robot = hsrb_interface.Robot()
    tf_buffer = robot._get_tf2_buffer()

    open_door = OpenDoor(robot=robot, handle_position='right',
                         timeout=1000, tf_buffer=tf_buffer)
    open_door.execute(userdata=None)
