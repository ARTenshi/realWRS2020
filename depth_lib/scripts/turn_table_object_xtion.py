#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import image_geometry
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Vector3Stamped
import os
from depth_lib import point_cloud, normal_image, smooth_normals

TABLE_NORM = [0,-1,0]
CAMERA_FRAME = 'camera_rgb_frame'
BASE_FRAME = 'camera_rgb_frame'
RGB_TOPIC = '/camera/rgb/image_rect_color'
DEPTH_TOPIC = '/camera/depth_registered/image_raw'
INFO_TOPIC = '/camera/rgb/camera_info'
MARGIN = 0.03

CAPTURE_RATE = 5.
MINIMUM_POINTS = 100
MINIMUM_WIDTH  = 50
MINIMUM_HEIGHT = 50

class TurnTableObjectFinder:
	def __init__(self, path):
		self.bridge = CvBridge()
		self.camera_model = image_geometry.PinholeCameraModel()
		self.tf_buffer = tf2_ros.Buffer()
		tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		rgb_sub = message_filters.Subscriber(RGB_TOPIC, Image)
		depth_sub = message_filters.Subscriber(DEPTH_TOPIC, Image)
		info_sub = message_filters.Subscriber(INFO_TOPIC, CameraInfo)
		self.sub = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 10, 1.)
		self.pub = rospy.Publisher('object_image', Image, queue_size=10)

		# Region of interest (left, right, top, bottom)
		self.RoI = None
		self._table_norm = TABLE_NORM
		self.clicked_point = None
		self.rate = rospy.Rate(CAPTURE_RATE)

		self.path = path
		self.count = 0
		dirname = os.path.dirname(path)
		basename = os.path.basename(path)
		files = filter(lambda x: x.startswith(basename) and x.endswith('.png'),
					   os.listdir(dirname))
		if files:
			nums = [f.strip('.png').split('_')[-1] for f in files if '_' in f]
			nums = [int(num) for num in nums if num.isdigit()]
			if nums:
				self.count = max(nums)+1
		self.stop()

	def table_norm(self):
		v = Vector3Stamped()
		v.vector.x = self._table_norm[0]
		v.vector.y = self._table_norm[1]
		v.vector.z = self._table_norm[2]
		v.header.stamp = rospy.Time.now()
		v.header.frame_id = BASE_FRAME
		try:
			v = self.tf_buffer.transform(v, CAMERA_FRAME,
										 rospy.Duration(1.))
		except Exception as e:
			rospy.logerr('Transformation error: '+str(e))
			return [0,-1,0]
		return [v.vector.x, v.vector.y, v.vector.z]

	def stop(self):
		self.sub.callbacks.clear()
		self.sub.registerCallback(self.callback_prep)
		cv2.namedWindow('Click', cv2.WINDOW_NORMAL)
		cv2.setMouseCallback('Click', self.mouse_event)

	def start(self):
		cv2.destroyWindow('Click')
		self.sub.callbacks.clear()
		self.rate = rospy.Rate(CAPTURE_RATE)
		self.sub.registerCallback(self.callback)

	def calc_points(self, depth, info):
		dimg = self.bridge.imgmsg_to_cv2(depth)
		#rospy.loginfo(dimg[np.isfinite(dimg)].max())
		self.camera_model.fromCameraInfo(info)

		P = self.camera_model.projectionMatrix()
		P = np.linalg.inv(P[:,:3])
		return point_cloud(dimg, P, self.RoI)

	def calc_alpha(self, depth, points):
		table_norm = self.table_norm()
		normals = normal_image(points)
		smooth_normals(normals)
		if np.isfinite(normals[self.clicked_point]).all():
			table_norm = normals[self.clicked_point]
			if np.dot(TABLE_NORM, table_norm):
				table_norm *= -1
		rospy.loginfo('table_norm = {}'.format(table_norm))
		d = np.dot(points, table_norm)
		D = d[self.clicked_point] + MARGIN
		P = points[self.clicked_point]
		thresh = depth.copy()
		thresh[d < D] = 0
		thresh[thresh > 2] = 0
		thresh[np.isnan(thresh)] = 0
		thresh = np.uint8((thresh != 0)*255)
		_, labels = cv2.distanceTransformWithLabels(255 - thresh,
													cv2.cv.CV_DIST_L2, 3)
		labels = labels & thresh
		if labels.max() <= 0:
			return None, None, None, None, None
		nearest = 1.
		for i in xrange(labels.max()):
			cond = labels == i+1
			if cond.sum() < MINIMUM_POINTS:
				continue
			dist = ((points[np.where(cond)] - P)**2).sum(1)
			dist = dist[np.isfinite(dist)].mean()
			if dist < nearest:
				nearest = dist
				nearest_i = i+1
		if nearest == 1.:
			return None, None, None, None, None

		alpha = np.uint8((labels == nearest_i)*255)
		dilate = cv2.dilate(alpha, np.ones((3,3), dtype=np.uint8))
		alpha = (alpha/255. + dilate/255.)/2.

		ys, xs = np.where(alpha != 0)
		t = ys.min()
		b = ys.max()
		l = xs.min()
		r = xs.max()
		if b-t < MINIMUM_HEIGHT or r-l < MINIMUM_WIDTH:
			return None, None, None, None, None
		return alpha, t, b, l, r

	def callback_prep(self, rgb, depth, info):
		depth, points = self.calc_points(depth, info)
		rgb = self.bridge.imgmsg_to_cv2(rgb, 'bgr8') #Assuming 'bgr8'

		cv2.imshow('Click', rgb)

		if self.clicked_point:
		  if np.isfinite(depth[self.clicked_point]):
			alpha, t, b, l, r = self.calc_alpha(depth, points)
			if alpha is None:
				return
			img = np.uint8(rgb*np.expand_dims(alpha,-1))
			cv2.rectangle(img, (l,t), (r,b), (0,0,255))
			cv2.imshow('Object', img)
		  else:
			rospy.loginfo('Clicked point is not finite.')
		cv2.waitKey(1)

	def callback(self, rgb, depth, info):
		depth, points = self.calc_points(depth, info)
		rgb = self.bridge.imgmsg_to_cv2(rgb) #Assuming 'bgr8'

		if self.clicked_point:
		  if np.isfinite(depth[self.clicked_point]):
			alpha, t, b, l, r = self.calc_alpha(depth, points)
			if alpha is None:
				return
			clip = np.uint8(np.dstack([rgb, alpha*255]))
			clip = clip[t:(b+1),l:(r+1),:]
			filename = self.path + '_%04d.png'%self.count
			#clip_BGR = cv2.cvtColor(clip, cv2.COLOR_RGB2BGR)
			cv2.imwrite(filename, clip)
			self.count += 1

			img = np.uint8(rgb*np.expand_dims(alpha,-1))
			cv2.rectangle(img, (l,t), (r,b), (0,0,255))
			cv2.imshow('Object', img)
			cv2.waitKey(1)
			self.rate.sleep()
		  else:
			rospy.loginfo('Clicked point is not finite.')

	def mouse_event(self, event, x, y, flags, param):
		if event != cv2.EVENT_LBUTTONUP:
			return
		rospy.loginfo('Mouse clicked.')
		self.clicked_point = (y,x)

if __name__ == '__main__':
	import sys
	path = sys.argv[1]
	rospy.init_node('turn_table_object')
	tto = TurnTableObjectFinder(path)
	rospy.loginfo('Images will be saved as "{}_xxxx.png."'.format(path))
	rospy.loginfo('Click the turn table and type enter to start.')
	while not rospy.is_shutdown():
		raw_input()
		if tto.clicked_point:
			break
		rospy.loginfo('Click the turn table before start.')
	tto.start()
	rospy.spin()
