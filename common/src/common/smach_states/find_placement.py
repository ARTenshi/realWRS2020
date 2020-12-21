import rospy
import smach
import traceback
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3Stamped, PointStamped

from depth_lib import read_point_cloud

# Adjust following values
# set them smaller if the resultant placement is too far from the target
# larger if the hand crashes onto the cupboard
KERNEL_WIDTH = 90
KERNEL_HEIGHT = 60


class FindPlacement(smach.State):
    def __init__(self, tf_buffer=None, timeout=None):
        #from depth_lib.srv import GetPerpendicularPlanes, GetParallelPlanes
        import tf2_ros
        import tf2_geometry_msgs
        from common import HSRB_Xtion
        smach.State.__init__(self, outcomes=['success', 'failure', 'no_space_found'],
                                   input_keys=['placement_hint', 'shelf_heights'],
                                   output_keys=['placement'])
        self.timeout = timeout

        # rospy.wait_for_service('get_perpendicular_planes')
        # rospy.wait_for_service('get_parallel_planes')
        #self.get_perpendicular_planes = rospy.ServiceProxy('get_perpendicular_planes', GetPerpendicularPlanes)
        #self.get_parallel_planes = rospy.ServiceProxy('get_parallel_planes', GetParallelPlanes)
        if tf_buffer is None:
            tf_buffer = tf2_ros.Buffer(rospy.Duration(20.))
            tf2_ros.TransformListener(tf_buffer)
        self.tf_buffer = tf_buffer
        self.xtion = HSRB_Xtion(tf_buffer=tf_buffer)

        self.placement_hint = PointStamped()
        self.placement_hint.header.frame_id = 'base_footprint'
        self.placement_hint.point.x = 0.5
        self.placement_hint.point.y = -0.2
        self.placement_hint.point.z = 0.5

    def execute(self, userdata):
        try:
            if userdata.placement_hint is not None:
                self.placement_hint = userdata.placement_hint
            else:
                rospy.logwarn('userdata.placement_hint is None. use default hint with best_shelf_height: ({}, {}, {})[frame={}]'.format(
                    self.placement_hint.point.x, self.placement_hint.point.y, self.placement_hint.point.z, self.placement_hint.header.frame_id))

            rospy.loginfo('placement_hint: ({}, {}, {})'.format(
                self.placement_hint.point.x, self.placement_hint.point.y, self.placement_hint.point.z))
            target_height = self.placement_hint.point.z

            shelf_heights = sorted(userdata.shelf_heights,
                                   key=lambda x: np.abs(x - target_height))
            shelf_indices = [userdata.shelf_heights.index(
                h) for h in shelf_heights]
            for j, best_shelf_height in enumerate(shelf_heights):
                rospy.loginfo(
                    'Looking at {}th shelf.'.format(shelf_indices[j]))
                i = userdata.shelf_heights.index(best_shelf_height)
                upper_shelf_height = shelf_heights[i +
                                                   1] if i+1 < len(shelf_heights) else h1+.5

                self.xtion.move(lift=best_shelf_height+0.4-1.0,
                                wait=True, timeout=self.timeout)
                self.xtion.look_at(
                    (.9, 0, best_shelf_height), 'base_footprint', wait=True, timeout=self.timeout)
                import time
                time.sleep(2.)
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
                        rospy.logerr(traceback.format_exc())
                        count += 1
                        if count > 5:
                            return 'failure'
                points, rgbimg = read_point_cloud(pointmsg)
                floor_norm = Vector3Stamped()
                floor_norm.header.frame_id = 'base_footprint'
                floor_norm.vector.z = 1.
                floor_norm = self.tf_buffer.transform(
                    floor_norm, pointmsg.header.frame_id, rospy.Duration(1.)).vector
                camera_pos = PointStamped()
                camera_pos.header.frame_id = pointmsg.header.frame_id
                camera_pos = self.tf_buffer.transform(
                    camera_pos, 'base_footprint', rospy.Duration(1.)).point
                print 'Best shelf: {}'.format(best_shelf_height)
                heights = points[:, :, 0] * floor_norm.x + points[:, :, 1] * floor_norm.y + points[:, :, 2] * floor_norm.z
                heights += camera_pos.z
                heights[np.isnan(heights)] = 0.
                space = np.logical_and(
                    heights > best_shelf_height-0.06, heights < best_shelf_height+0.03)
                #rgbimg[np.where(space)] = (255, 0, 0)
                #cv2.imshow('Shelf', rgbimg)
                # cv2.waitKey()
                #kernel = np.ones((KERNEL_HEIGHT,KERNEL_WIDTH),np.float32)/(KERNEL_WIDTH*KERNEL_HEIGHT)
                #space = cv2.filter2D(np.uint8(space)*255,-1,kernel) > 250
                kernel = np.ones((KERNEL_HEIGHT, KERNEL_WIDTH), dtype=np.uint8)
                space = cv2.erode(np.uint8(space)*255, kernel, iterations=1)
                target_cam = self.tf_buffer.transform(
                    self.placement_hint, pointmsg.header.frame_id, rospy.Duration(1.)).point
                target_cam = np.array(
                    [[[target_cam.x, target_cam.y, target_cam.z]]])
                distances = np.sqrt(((points - target_cam)**2).sum(2))
                score = 1./distances
                score[np.logical_not(space)] = 0.
                score[np.isnan(score)] = 0.
                best_uv = np.unravel_index(score.argmax(), score.shape)

                print 'Placement height: {}'.format(heights[best_uv])
                print 'upper shelf height: {}'.format(upper_shelf_height)
                rgbimg[np.where(space)] = (255, 0, 0)
                rgbimg[best_uv] = (0, 0, 255)
                #cv2.imshow('Shelf', rgbimg)
                # cv2.waitKey()
                placement = PointStamped()
                if space.sum() == 0 and j == len(shelf_heights)-1:
                    rospy.logerr('There are no space to place an item.')
                    placement.header.frame_id = userdata.placement_hint.header.frame_id
                    placement.point = userdata.placement_hint.point
                    placement = self.tf_buffer.transform(
                        placement, 'base_footprint', rospy.Duration(1.))
                    userdata.placement = placement
                    return 'no_space_found'
                if space.sum() != 0:
                    break

            placement.header.frame_id = pointmsg.header.frame_id
            placement.point.x, placement.point.y, placement.point.z = points[best_uv]
            placement = self.tf_buffer.transform(
                placement, 'base_footprint', rospy.Duration(1.))
            if placement.point.z-best_shelf_height < 0.05:
                placement.point.z = best_shelf_height + 0.1
            #placement.point.z += min(.1, (upper_shelf_height-best_shelf_height)/2)

            print 'Best placement: {}'.format(placement.point)
            if placement.point.x > 0.95:
                placement.point.x = 0.95
            if placement.point.y < -0.25:
                placement.point.y = -0.25
            if placement.point.y > 0.25:
                placement.y = 0.25
            print 'Corrected best placement: {}'.format(placement.point)
            #print placement
            userdata.placement = placement
            return 'success'
        except:
            rospy.logerr(traceback.format_exc())
            return 'failure'
