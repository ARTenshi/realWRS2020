#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import image_geometry
import tf2_ros
import tf2_geometry_msgs
from threading import BoundedSemaphore

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3Stamped, Pose, Quaternion

from depth_lib import morph_kernel
from depth_lib import point_cloud, normal_image, smooth_normals, convert_transform, gripper_pose, cluster_planes, refine_planes, largest_region, closest_region, lines_3d
from depth_lib.msg import Object, Plane
from depth_lib.srv import GetPerpendicularPlanes, GetParallelPlanes, GetObject, GetTableTopObject, GetTray, GetDish, GetBag, GetDoorHandle
from depth_lib.srv import GetBagResponse, GetPerpendicularPlanesResponse, GetParallelPlanesResponse, GetObjectResponse, GetTableTopObjectResponse, GetTrayResponse, GetDoorHandleResponse

class DepthLib:
	def __init__(self):
		self.gripper_frame = 'hand_palm_link' #TODO: parameter
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
		tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		self.rgbmsg = None
		self.depthmsg = None
		self.normals = None
		self.sema_norm = BoundedSemaphore(1)
		self.bridge = CvBridge()
		self.camera_model = image_geometry.PinholeCameraModel()
		rgb_sub = message_filters.Subscriber('rgb_image', Image)
		depth_sub = message_filters.Subscriber('depth_image', Image)
		info_sub = rospy.Subscriber('camera_info', CameraInfo, self.info_cb)
		self.sub = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 1.)
		self.sub.registerCallback(self.callback)

		#rgb_sub.registerCallback(self.dbg_cb)
		#depth_sub.registerCallback(self.dbg_cb)
		#self.sub.registerCallback(self.dbg_cb)

		self.pub_norm = rospy.Publisher('normal_image', Image, queue_size=10)
		self.pub = rospy.Publisher('plane_candidates', Image, queue_size=10)

		rospy.Service('get_perpendicular_planes', GetPerpendicularPlanes, self.getPerpendicularPlanes)
		rospy.Service('get_parallel_planes', GetParallelPlanes, self.getParallelPlanes)
		rospy.Service('get_object', GetObject, self.getObject)
		rospy.Service('get_table_top_object', GetTableTopObject, self.getTableTopObject)
		rospy.Service('get_tray', GetTray, self.getTray)
		rospy.Service('get_dish', GetDish, self.getDish)
		rospy.Service('get_bag', GetBag, self.getBag)
		rospy.Service('get_door_handle', GetDoorHandle, self.getDoorHandle)

	#def dbg_cb(self, *msgs):
	#    rospy.loginfo(map(type,msgs))

	def calc_normals(self, rect=None):
		with self.sema_norm:
			if self.normals is not None:
				left, right, top, bottom = rect if rect is not None else (0, -1, 0, -1)
				left = int(left)
				right = int(right)
				top = int(top)
				bottom = int(bottom)
				depth = self.depth[top:bottom, left:right]
				points = self.points[top:bottom, left:right, :]
				normals = self.normals[top:bottom, left:right, :]
				return depth, points, normals

		now = rospy.Time.now()
		count = 0
		while not rospy.is_shutdown():
			if self.depthmsg and (now - self.depthmsg.header.stamp).to_sec() < 1.0:
				break
			rospy.logwarn('Data is too old.')
			try:
				self.depthmsg = rospy.wait_for_message('depth_image', Image, .5)
			except:
				count += 1
				if count > 5:
					rospy.logerr('No message arrived.')
					return None
		img = self.bridge.imgmsg_to_cv2(self.depthmsg)
		# In openni, depth values seem to be in mm, whereas in openni2 they are in m.
		#img = img/1000.

		P = self.camera_model.projectionMatrix()
		P = np.linalg.inv(P[:,:3])
		depth, points = point_cloud(img, P, rect)
		normals = -normal_image(points)
		smooth_normals(normals)
		return depth, points, normals

	# get transform to depth frame as rotation matrix and translation vector
	def get_transform(self, parent_frame):
		tf = self.tf_buffer.lookup_transform(parent_frame,
											 self.depthmsg.header.frame_id,
											 self.depthmsg.header.stamp,
											 rospy.Duration(1.))
		return convert_transform(tf)

	def maybe_publish_labels(self, labels):
		if self.pub.get_num_connections() > 0:
			label_normal = cv2.normalize(labels, alpha=0, beta=255, norm_type=cv2.CV_MINMAX)
			label_color = cv2.applyColorMap(np.uint8(label_normal), cv2.COLORMAP_JET)
			self.pub.publish(self.bridge.cv2_to_imgmsg(label_color))

	def getBag(self, req):
		morph_kernel3 = np.array([[0, 0, 1, 0, 0],
								  [0, 0, 1, 0, 0],
								  [1, 1, 1, 1, 1],
								  [0, 0, 1, 0, 0],
								  [0, 0, 1, 0, 0]], np.uint8)
		morph_kernel_vert = np.ones((15,5), np.uint8)

		result = self.calc_normals()
		if result is None:
			return GetBagResponse()
		depth, points, normals = result

		maxval = depth[np.isfinite(depth)].max()
		img = depth.copy()
		img[img > 1.5] = maxval
		img[np.isnan(img)] = maxval
		img = 255-np.uint8(img/maxval*255)
		ret, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_OTSU)
		tophat = cv2.erode(thresh, morph_kernel, iterations=2)
		tophat = cv2.morphologyEx(tophat, cv2.MORPH_TOPHAT, morph_kernel3)
		tophat = cv2.dilate(tophat, morph_kernel_vert, iterations=1)
		#tophat = cv2.morphologyEx(tophat, cv2.MORPH_OPEN, morph_kernel2)
		_, labels = cv2.distanceTransformWithLabels(255-tophat, cv2.DIST_L2, 3)
		labels = labels & tophat
		nums = [(labels==i).sum() for i in range(labels.max()+1)]
		i,j = np.argsort(nums[1:])[-2:]
		labels[np.logical_and(labels != i+1, labels != j+1)] = 0
		self.maybe_publish_labels(labels)

		X = points[np.where(labels==i+1)]
		Y = points[np.where(labels==j+1)]
		if X.shape[0] == 0 or Y.shape[0] == 0:
			return GetBagResponse()
		p1 = X[np.where(np.isfinite(X))[0]].mean(0)
		p2 = Y[np.where(np.isfinite(Y))[0]].mean(0)
		if np.linalg.norm(p1-p2) > .15:
			return GetBagResponse()

		grasp_pos = p1
		open_dir = p1-p2
		open_dir /= np.linalg.norm(open_dir)
		if open_dir[1] < 0:
			open_dir = -open_dir
		if open_dir[0] < 0:
			open_dir = -open_dir
		aim_dir = [0,0,1]

		res = GetBagResponse()
		pose1, pose2 = gripper_pose(grasp_pos, open_dir, aim_dir)
		res.object.graspPose.pose = pose2
		res.object.graspPose.header.stamp = self.depthmsg.header.stamp
		res.object.graspPose.header.frame_id = self.gripper_frame
		res.object.approachPose.pose = pose1
		res.object.approachPose.header.stamp = self.depthmsg.header.stamp
		res.object.approachPose.header.frame_id = self.depthmsg.header.frame_id
		res.object.graspWidth = .3
		return res

	def getPerpendicularPlanes(self, req):
		result = self.calc_normals()
		if result is None:
			return GetPerpendicularPlanesResponse()
		depth, points, normals = result
		rot, tr = self.get_transform(req.axis.header.frame_id)
		plane_norm = [req.axis.vector.x,
					  req.axis.vector.y,
					  req.axis.vector.z]
		plane_norm = np.dot(rot, plane_norm)
		cos = np.dot(normals, plane_norm)
		ret, thresh = cv2.threshold(np.uint8((cos+1)/2*255),
									230, 255, cv2.THRESH_BINARY)
		labels = cluster_planes(depth, thresh)
		#self.maybe_publish_labels(labels)
		params, centers = refine_planes(points, labels,
										min_points=req.minimum_points,
										n_samples=req.iterations,
										max_dist=req.thickness,
										inlier_ratio=req.minimum_inlier_ratio)
		self.maybe_publish_labels(labels)
		n_planes = labels.max()
		res = GetPerpendicularPlanesResponse()
		res.planes = [Plane() for _ in xrange(n_planes)]
		for i in xrange(n_planes):
			res.planes[i].normal.vector.x = params[i+1,0]
			res.planes[i].normal.vector.y = params[i+1,1]
			res.planes[i].normal.vector.z = params[i+1,2]
			res.planes[i].normal.header.stamp = self.depthmsg.header.stamp
			res.planes[i].normal.header.frame_id = self.depthmsg.header.frame_id
			res.planes[i].center.point.x = centers[i+1,0]
			res.planes[i].center.point.y = centers[i+1,1]
			res.planes[i].center.point.z = centers[i+1,2]
			res.planes[i].center.header.stamp = self.depthmsg.header.stamp
			res.planes[i].center.header.frame_id = self.depthmsg.header.frame_id
		return res

	def getParallelPlanes(self, req):
		result = self.calc_normals()
		if result is None:
			return GetParallelPlanesResponse()
		depth, points, normals = result
		rot, tr = self.get_transform(req.axis.header.frame_id)
		plane_norm = [req.axis.vector.x,
					  req.axis.vector.y,
					  req.axis.vector.z]
		plane_norm = np.dot(rot, plane_norm)
		cos = np.dot(normals, plane_norm)
		sin = np.sqrt(1-cos*cos)
		ret, thresh = cv2.threshold(np.uint8((sin+1)/2*255),
									230, 255, cv2.THRESH_BINARY)
		labels = cluster_planes(depth, thresh)
		#self.maybe_publish_labels(labels)
		params, centers = refine_planes(points, labels,
										min_points=req.minimum_points,
										n_samples=req.iterations,
										max_dist=req.thickness,
										inlier_ratio=req.minimum_inlier_ratio)
		self.maybe_publish_labels(labels)
		n_planes = labels.max()
		res = GetParallelPlanesResponse()
		res.planes = [Plane() for _ in xrange(n_planes)]
		for i in xrange(n_planes):
			res.planes[i].normal.vector.x = params[i+1,0]
			res.planes[i].normal.vector.y = params[i+1,1]
			res.planes[i].normal.vector.z = params[i+1,2]
			res.planes[i].normal.header.stamp = self.depthmsg.header.stamp
			res.planes[i].normal.header.frame_id = self.depthmsg.header.frame_id
			res.planes[i].center.point.x = centers[i+1,0]
			res.planes[i].center.point.y = centers[i+1,1]
			res.planes[i].center.point.z = centers[i+1,2]
			res.planes[i].center.header.stamp = self.depthmsg.header.stamp
			res.planes[i].center.header.frame_id = self.depthmsg.header.frame_id
		return res

	def getObject(self, req):
		rgbimg = self.bridge.imgmsg_to_cv2(self.rgbmsg)
		rgbimg = rgbimg[max(0,int(req.top)):int(req.bottom),
						max(0,int(req.left)):int(req.right), :]
		rect = (req.left, req.right, req.top, req.bottom)
		result = self.calc_normals(rect)
		if result is None:
			return GetObjectResponse()
		depth, points, normals = result
		rot, tr = self.get_transform('base_footprint')
		# Attempt to find a surface that faces the robot.
		# If you want to find one that faces the camera,
		# use norm=[0,0,1] without transform.
		norm = np.dot(rot, [-1,0,0])
		cos = np.dot(normals, norm)
		ret, thresh = cv2.threshold(np.uint8((cos+1)/2*255),
									0, 255, cv2.THRESH_OTSU)
		labels = cluster_planes(depth, thresh)
		if labels.max() == 0: # no region found
			return GetObjectResponse()
		i = largest_region(labels)
		labels[labels != i] = 0
		self.maybe_publish_labels(labels)
		inliers = labels==i
		x = points[:,:,0][inliers]
		y = points[:,:,1][inliers]
		z = points[:,:,2][inliers]
		obj = Object()
		obj.center.header.stamp = self.depthmsg.header.stamp
		obj.center.header.frame_id = self.depthmsg.header.frame_id
		obj.center.point.x = x.mean()
		obj.center.point.y = y.mean()
		obj.center.point.z = z.mean()
		_, cont, _ = cv2.findContours(np.uint8(inliers*255),
								   cv2.RETR_EXTERNAL,
								   cv2.CHAIN_APPROX_SIMPLE)
		center, size, angle = cv2.minAreaRect(cont[0])
		if size[0] > size[1]:
			th = (angle + 90.)*np.pi/180.
			width = size[1]
		else:
			th = angle*np.pi/180.
			width = size[0]
		open_dir = np.array([np.cos(th), np.sin(th), 0],
							dtype=np.float32)
		aim_dir = np.array([0,0,1])
		grasp_pos = points[int(center[1]), int(center[0])]
		if np.dot(open_dir , rot[:,2]) < -0.5\
		   or np.dot(open_dir, rot[:,1]) > 0.5: # if sucker up or right
			open_dir = -open_dir
		pose1, pose2 = gripper_pose(grasp_pos, open_dir, aim_dir)
		obj.graspPose.pose = pose2
		obj.graspPose.header.stamp = self.depthmsg.header.stamp
		obj.graspPose.header.frame_id = self.gripper_frame
		obj.approachPose.pose = pose1
		obj.approachPose.header.stamp = self.depthmsg.header.stamp
		obj.approachPose.header.frame_id = self.depthmsg.header.frame_id
		obj.graspWidth = width
		mask = np.uint8(inliers*255)
		obj.colorHistogram = cv2.calcHist([rgbimg], [0], mask, [180], [0,180]).reshape(-1)
		return GetObjectResponse(obj)

	def getTableTopObject(self, req):
		rgbimg = self.bridge.imgmsg_to_cv2(self.rgbmsg)
		rgbimg = rgbimg[max(0,int(req.top)):int(req.bottom),
						max(0,int(req.left)):int(req.right), :]
		rect = (req.left, req.right, req.top, req.bottom)
		height = req.table_height
		result = self.calc_normals(rect)
		if result is None:
			return GetTableTopObjectResponse()
		depth, points, normals = result
		rot, tr = self.get_transform('base_footprint')
		# Attempt to find a surface that faces the robot.
		# If you want to find one that faces the camera,
		# use norm=[0,0,1] without transform.
		norm = np.dot(rot, [-1,0,0])
		cos = np.dot(normals, norm)
		ret, thresh = cv2.threshold(np.uint8((cos+1)/2*255),
									0, 255, cv2.THRESH_OTSU)
		labels = cluster_planes(depth, thresh)
		if labels.max() == 0: # no region found
			return GetTableTopObjectResponse()
		i = largest_region(labels)
		labels[labels != i] = 0
		self.maybe_publish_labels(labels)
		inliers = labels==i
		x = points[:,:,0][inliers]
		y = points[:,:,1][inliers]
		z = points[:,:,2][inliers]
		obj = Object()
		obj.center.header.stamp = self.depthmsg.header.stamp
		obj.center.header.frame_id = self.depthmsg.header.frame_id
		obj.center.point.x = x.mean()
		obj.center.point.y = y.mean()
		obj.center.point.z = z.mean()
		_, cont, _ = cv2.findContours(np.uint8(inliers*255),
								   cv2.RETR_EXTERNAL,
								   cv2.CHAIN_APPROX_SIMPLE)
		cx,cy,width,_ = cv2.boundingRect(cont[0])
		open_dir = -rot[:,1]
		aim_dir = rot[:,0]
		#grasp_pos = points[int(cy), int(cx)]
		grasp_pos = [obj.center.point.x, obj.center.point.y, obj.center.point.z]
		pose1, pose2 = gripper_pose(grasp_pos, open_dir, aim_dir, back_dist=.2)
		obj.graspPose.pose = pose2
		obj.graspPose.header.stamp = self.depthmsg.header.stamp
		obj.graspPose.header.frame_id = self.gripper_frame
		obj.approachPose.pose = pose1
		obj.approachPose.header.stamp = self.depthmsg.header.stamp
		obj.approachPose.header.frame_id = self.depthmsg.header.frame_id
		obj.graspWidth = width
		mask = np.uint8(inliers*255)
		obj.colorHistogram = cv2.calcHist([rgbimg], [0], mask, [180], [0,180]).reshape(-1)
		return GetTableTopObjectResponse(obj)

	def getTray(self, req):
		result = self.calc_normals()
		if result is None:
			return GetTrayResponse()
		depth, points, normals = result
		rot, tr = self.get_transform('base_footprint')
		norm = np.dot(rot, [0,0,1])
		cos = np.dot(normals, norm)
		ret, thresh = cv2.threshold(np.uint8((cos+1)/2*255),
									230, 255, cv2.THRESH_BINARY)
		labels = cluster_planes(depth, thresh)

		cam_height = tr[2]
		heights = np.dot(points, norm) + cam_height
		l2 = 1
		plane_heights = []
		for l1 in xrange(1,labels.max()+1):
			inds = labels==l1
			h = heights[inds & np.isfinite(heights)].mean()
			if h > .5 and h < 1.5 and inds.sum() > 1000:
				labels[inds] = l2
				plane_heights.append(h)
				l2 += 1
			else:
				labels[inds] = 0

		if labels.max() == 0:
			return GetTrayResponse()
		i = closest_region(points, labels)
		labels[labels != i] = 0
		labels[labels == i] = 1
		refine_planes(points, labels, max_dist=1)
		plane_height = plane_heights[i-1]

		labels8 = np.uint8(labels*255)
		#labels8d = cv2.dilate(labels8, morph_kernel, iterations=15)
		labels8 = labels8 & np.uint8((heights>plane_height)*255)
		labels8 = labels8 & np.uint8((cos < .7)*255)
		labels8 = cv2.morphologyEx(labels8, cv2.MORPH_OPEN,
								   morph_kernel, iterations=1)
		self.maybe_publish_labels(labels8)

		lines = cv2.HoughLinesP(labels8,1,np.pi/180,100,50,5)
		if lines is None:
			return GetTrayResponse()
		'''
		rect = extract_rectangle(lines[0], points)
		if rect is None:
			return GetTrayResponse()
		for i, p in enumerate(rect):
			res.endPoints[i].point.x = p[0]
			res.endPoints[i].point.y = p[1]
			res.endPoints[i].point.z = p[2]
			res.endPoints[i].header.frame_id = self.depthmsg.header.frame_id
			res.endPoints[i].header.stamp = self.depthmsg.header.stamp
		'''
		centers, dirs, lengths = lines_3d(lines[0], points)
		horiz = np.dot(dirs, rot[:,1])**2 > .5
		if horiz.sum() == 0:
			return GetTrayResponse()
		centers = centers[np.where(horiz)]
		dirs = dirs[np.where(horiz)]
		i = centers[:,2].argmin()

		res = GetTrayResponse()
		open_dir = np.cross(norm, dirs[i])
		if np.dot(open_dir , rot[:,2]) < -0.5\
		   or np.dot(open_dir, rot[:,1]) > 0.5: # if sucker up or right
			open_dir = -open_dir
		pose1, pose2 = gripper_pose(centers[i], open_dir, -norm)
		res.object.graspPose.pose = pose2
		res.object.graspPose.header.frame_id = self.gripper_frame
		res.object.graspPose.header.stamp = self.depthmsg.header.stamp
		res.object.approachPose.pose = pose1
		res.object.approachPose.header.frame_id = self.depthmsg.header.frame_id
		res.object.approachPose.header.stamp = self.depthmsg.header.stamp
		return res
		
	def getDoorHandle(self, req):
		result = self.calc_normals(rect=(200,440,0,480))
		if result is None:
			return GetDoorHandleResponse()
		depth, points, normals = result

		rot, tr = self.get_transform('base_footprint')
		norm = np.dot(rot, [-1,0,0])
		floor_norm = np.dot(rot, [0,0,1])
		cos = np.dot(normals, norm)
		ret, thresh = cv2.threshold(np.uint8((cos+1)/2*255),
									0, 255, cv2.THRESH_OTSU)
		labels = cluster_planes(depth, thresh)
		#refine_planes(points, labels)
		if labels.max() == 0:
			return GetDoorHandleResponse()
		i = largest_region(labels)

		labels[labels != i] = 0
		labels8 = np.uint8((labels==i)*255)
		#labels8 = cv2.erode(labels8, morph_kernel, iterations=15)
		inds = labels8 == 255
		nx = normals[:,:,0][inds]
		nx = nx[np.isfinite(nx)].mean()
		ny = normals[:,:,1][inds]
		ny = ny[np.isfinite(ny)].mean()
		nz = normals[:,:,2][inds]
		nz = nz[np.isfinite(nz)].mean()
		plane_norm = np.array((nx,ny,nz))/np.sqrt(nx*nx+ny*ny+nz*nz)
		dists = np.dot(points, plane_norm)
		inds = np.logical_and(np.isfinite(dists), labels==i)
		plane_dist = dists[inds].mean()

		cam_height = tr[2]
		heights = np.dot(points, floor_norm) + cam_height

		cond = dists > plane_dist+.02
		cond = np.logical_and(cond, heights > .8)
		cond = np.logical_and(cond, heights < 1.1)
		cond8 = np.uint8(cond*255)
		self.maybe_publish_labels(cond8 & labels8)
		inds = (cond8 & labels8) == 255
		cx = points[:,:,0][inds]
		cx = cx[np.isfinite(cx)].mean()
		cy = points[:,:,1][inds]
		cy = cy[np.isfinite(cy)].mean()
		cz = points[:,:,2][inds]
		cz = cz[np.isfinite(cz)].mean()
		res = GetDoorHandleResponse()
		pose1, pose2 = gripper_pose((cx,cy,cz), floor_norm, -plane_norm)
		res.object.graspPose.pose = pose2
		res.object.graspPose.header.frame_id = self.gripper_frame
		res.object.graspPose.header.stamp = self.depthmsg.header.stamp
		res.object.approachPose.pose = pose1
		res.object.approachPose.header.frame_id = self.depthmsg.header.frame_id
		res.object.approachPose.header.stamp = self.depthmsg.header.stamp
		res.object.center.point = res.object.graspPose.pose.position
		res.object.center.header = res.object.graspPose.header
		res.object.graspWidth = .3
		return res

	def getDish(self, req):
		# STUB service callback
		result = self.calc_normals()
		if result is None:
			return None
		depth, points, normals = result
		#TODO: publish objects

	def info_cb(self, info):
		self.camera_model.fromCameraInfo(info)

	def callback(self, rgbmsg, depthmsg):
		if (rospy.Time.now() - rgbmsg.header.stamp).to_sec() > 1.:
			return
		rospy.loginfo('Message arrived.')
		self.rgbmsg = rgbmsg
		self.depthmsg = depthmsg
		if self.pub_norm.get_num_connections() == 0:
			with self.sema_norm:
				self.depth = None
				self.points = None
				self.normals = None
			return

		depthimg = self.bridge.imgmsg_to_cv2(self.depthmsg)
		#depthimg = depthimg/1000.

		P = self.camera_model.projectionMatrix()
		P = np.linalg.inv(P[:,:3])
		depth, points = point_cloud(depthimg, P)
		normals = -normal_image(points)
		smooth_normals(normals)
		with self.sema_norm:
			self.depth = depth
			self.points = points
			self.normals = normals
		self.pub_norm.publish(self.bridge.cv2_to_imgmsg(normals))

if __name__ == '__main__':
	rospy.init_node('depth_lib')
	DepthLib()
	rospy.spin()
