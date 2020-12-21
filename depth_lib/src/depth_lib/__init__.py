#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Vector3Stamped, Pose, Quaternion
from depth_lib.msg import Object, Plane
from depth_lib.srv import GetBagResponse, GetPerpendicularPlanesResponse, GetParallelPlanesResponse, GetObjectResponse, GetTableTopObjectResponse, GetTrayResponse
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import traceback

dx_filter = np.array([[-1, 1],
					  [-1, 1]], np.float32)
dy_filter = np.array([[-1,-1],
					  [ 1, 1]], np.float32)
morph_kernel = np.array([[0, 1, 1, 0],
						 [1, 1, 1, 1],
						 [1, 1, 1, 1],
						 [0, 1, 1, 0]], np.uint8)

def read_point_cloud(pointmsg):
	assert len(pointmsg.fields)==4 and pointmsg.fields[3].name=='rgb' and pointmsg.point_step==32,\
		"Assuming 'x','y','z','rgb' point fields in this order. Use OpenNI."

	if pointmsg.row_step == pointmsg.point_step * pointmsg.width:
		data = np.fromstring(pointmsg.data, dtype=np.float32).reshape(pointmsg.height, pointmsg.width, 8)
	else:
		data = np.zeros((pointmsg.height, pointmsg.width, 8), dtype=np.float32)
		for v in range(pointmsg.height):
			begin = pointmsg.row_step * v
			end = pointmsg.row_step * (v+1)
			data[v,:] = np.fromstring(pointmsg.data[begin:end], dtype=np.float32).reshape(pointmsg.width, 4)

	points = data[:,:,:3]
	rgbdata = data[:,:,4].view(dtype=np.uint32)
	rimg = np.uint8((rgbdata & 0xff0000) >> 16)
	gimg = np.uint8((rgbdata & 0xff00) >> 8)
	bimg = np.uint8(rgbdata & 0xff)
	rgbimg = np.dstack([bimg,gimg,rimg])
	return points, rgbimg

def largest_region(labels):
	if labels.max() <= 0:
		return None
	nums = [np.sum(labels==i) for i in xrange(1,labels.max()+1)]
	return np.array(nums).argmax()+1

def closest_region(points, labels):
	if labels.max() <= 0:
		return None
	dist = np.zeros(labels.max())
	for i in xrange(labels.max()):
		#inds = labels==i+1
		#x = points[:,:,0][inds]
		#y = points[:,:,1][inds]
		#z = points[:,:,2][inds]
		#dist[i] = np.sqrt(x*x+y*y+z*z).mean()
		inliers = points[np.where(labels==i+1)]
		dist[i] = np.sqrt((inliers**2).sum(1)).mean()
	return np.array(dist).argmin()+1

def make_pose(pos, norm):
	pose = Pose()
	pose.position.x = pos[0]
	pose.position.y = pos[1]
	pose.position.z = pos[2]
	#s=np.sqrt((1-cos)/(2-2*cos*cos))
	s = np.sqrt(2*(1+norm[0]))
	pose.orientation.w = s/2
	pose.orientation.x = 0
	pose.orientation.y = -norm[2]/s
	pose.orientation.z = norm[1]/s
	return pose

def matrix_to_quaternion(mat):
	m00 = mat[0,0]
	m11 = mat[1,1]
	m22 = mat[2,2]
	if m00+m11+m22 > 0:
		qw = np.sqrt(m00+m11+m22+1.)/2
		qx = (mat[2,1]-mat[1,2])/(4*qw)
		qy = (mat[0,2]-mat[2,0])/(4*qw)
		qz = (mat[1,0]-mat[0,1])/(4*qw)
	elif m00 > m11 and m00 > m22:
		qx = np.sqrt(m00-m11-m22+1.)/2
		qw = (mat[2,1]-mat[1,2])/(4*qx)
		qy = (mat[1,0]+mat[0,1])/(4*qx)
		qz = (mat[0,2]+mat[2,0])/(4*qx)
	elif m11 > m22:
		qy = np.sqrt(m11-m00-m22+1.)/2
		qw = (mat[0,2]-mat[2,0])/(4*qy)
		qx = (mat[1,0]+mat[0,1])/(4*qy)
		qz = (mat[2,1]+mat[1,2])/(4*qy)
	else:
		qz = np.sqrt(m22-m00-m11+1.)/2
		qw = (mat[1,0]-mat[0,1])/(4*qz)
		qx = (mat[0,2]+mat[2,0])/(4*qz)
		qy = (mat[2,1]+mat[1,2])/(4*qz)
	q = Quaternion()
	q.w = qw
	q.x = qx
	q.y = qy
	q.z = qz
	return q

def quaternion_to_matrix(q):
	m11 = 1.0 - 2.0 * q.y * q.y - 2.0 * q.z * q.z
	m12 = 2.0 * q.x * q.y + 2.0 * q.w * q.z
	m13 = 2.0 * q.x * q.z - 2.0 * q.w * q.y
	m21 = 2.0 * q.x * q.y - 2.0 * q.w * q.z
	m22 = 1.0 - 2.0 * q.x * q.x - 2.0 * q.z * q.z
	m23 = 2.0 * q.y * q.z + 2.0 * q.w * q.x
	m31 = 2.0 * q.x * q.z + 2.0 * q.w * q.y
	m32 = 2.0 * q.y * q.z - 2.0 * q.w * q.x
	m33 = 1.0 - 2.0 * q.x * q.x - 2.0 * q.y * q.y
	return np.array([[m11, m12, m13],
					 [m21, m22, m23],
					 [m31, m32, m33]], dtype=np.float32)

# grasp_pos: point of grasp
# open_dir: axis along which the gripper opens
# (gripper with sucker -> the other)
# aim_dir: direction of the gripper
# (gripper -> grasp_pos)
# returns: pose of gripper (the second one is relative to the first one)
def gripper_pose(grasp_pos, open_dir, aim_dir, back_dist=.1):
	grasp_pos = np.float32(grasp_pos)
	open_dir = np.float32(open_dir)
	aim_dir = np.float32(aim_dir)

	y_axis = np.array(open_dir)
	x_axis = np.cross(y_axis, aim_dir)
	z_axis = np.cross(x_axis, y_axis)
	x_axis /= np.linalg.norm(x_axis)
	y_axis /= np.linalg.norm(y_axis)
	z_axis /= np.linalg.norm(z_axis)
	# rotation matrix to quaternion

	pose1 = Pose()
	pose1.position.x = grasp_pos[0] - z_axis[0] * back_dist
	pose1.position.y = grasp_pos[1] - z_axis[1] * back_dist
	pose1.position.z = grasp_pos[2] - z_axis[2] * back_dist
	pose1.orientation = matrix_to_quaternion(np.vstack([x_axis, y_axis, z_axis]).T)

	pose2 = Pose()
	pose2.position.z = back_dist
	pose2.orientation.w = 1.

	return pose1, pose2

def normal_image(points):
	x = points[:,:,0]
	y = points[:,:,1]
	z = points[:,:,2]
	dxdu = cv2.filter2D(x, -1, dx_filter)
	dxdv = cv2.filter2D(x, -1, dy_filter)
	dydu = cv2.filter2D(y, -1, dx_filter)
	dydv = cv2.filter2D(y, -1, dy_filter)
	dzdu = cv2.filter2D(z, -1, dx_filter)
	dzdv = cv2.filter2D(z, -1, dy_filter)
	dzdu = cv2.GaussianBlur(dzdu, (3,3), 0)
	dzdv = cv2.GaussianBlur(dzdv, (3,3), 0)
	#l = np.sqrt(dzdx**2 + dzdy**2)
	#return np.stack([dzdx/l, dzdy/l], 2)
	norm = np.dstack([dydu*dzdv-dzdu*dydv,
					  dzdu*dxdv-dxdu*dzdv,
					  dxdu*dydv-dydu*dxdv])
	l = np.sqrt(np.sum(norm*norm, 2))
	return norm/l.reshape(l.shape[0],l.shape[1],1)

def point_cloud(img, P, rect=None):
	if rect:
		left, right, top, bottom = rect
		left = max(0, int(left))
		top = max(0, int(top))
		right = min(img.shape[1], int(right))
		bottom = min(img.shape[0], int(bottom))
	else:
		left = 0
		top = 0
		right = img.shape[1]
		bottom = img.shape[0]
	img = img[top:bottom, left:right]
	X,Y = np.meshgrid(xrange(left, right), xrange(top, bottom))
	points = np.array(np.dot(P, np.vstack([X.flat,Y.flat,np.ones_like(X.flat)])))
	points *= img.reshape(1,-1)
	points = np.reshape(points, (-1,)+img.shape)
	points = np.transpose(points, (1,2,0))
	return img, points

def smooth_normals(normals):
	src = np.uint8((normals+1)/2*255)
	dst = np.zeros_like(src)
	cv2.pyrMeanShiftFiltering(src, 5., 50., dst)
	fin_ind = np.isfinite(normals)
	normals[fin_ind] = dst[fin_ind]/255.*2-1
	# re-normalize
	shape = normals.shape
	normals /= np.sqrt(np.sum(normals*normals,2)).reshape(shape[0],shape[1],1)

def cluster_planes(depth, thresholded, open_kernel=morph_kernel):
	valid_depth = depth[np.isfinite(depth)]
	if valid_depth.size > 0:
		maximum = valid_depth.max()
	else:
		maximum = np.inf
	depth8 = np.uint8(depth/maximum*255)
	canny = cv2.Canny(depth8, 10, 30)
	canny = cv2.dilate(canny, morph_kernel, iterations=2)
	#opening = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, morph_kernel, iterations=3)
	#opening = cv2.erode(opening, morph_kernel, iterations=3)
	opening = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, open_kernel, iterations=1)
	opening = cv2.dilate(opening, morph_kernel, iterations=1)
	region = (255-opening) | canny
	_, labels = cv2.distanceTransformWithLabels(region, cv2.DIST_L2, 3)
	return labels & (255-region)

def refine_planes(points, labels, min_points=1000, n_samples=15,
				  max_dist=0.02, inlier_ratio=.2, convex_hull=True):
	planes = np.zeros((1,4))
	centers = np.zeros((1,3))
	l2 = 1
	for l1 in xrange(1, labels.max()+1):
		inds = labels==l1
		N = inds.sum()
		if N < min_points:
			labels[inds] = 0
			continue
		X = points[np.where(inds)]

		max_inliers = None
		max_n_inliers = 0
		max_plane = None
		for i in xrange(n_samples):
			n = None
			j = 0
			while n is None:
				try:
					#P = X[np.random.choice(N,3)]
					#n = np.linalg.solve(P, [1,1,1])
					P = X[np.random.choice(N,20)]
					n = np.linalg.solve(np.dot(P.T,P), P.sum(0))
				except Exception as e:
					j += 1
					if j > 5:
						break
			if n is None:
				continue
			n /= np.linalg.norm(n)
			if n[2] > 0:
				n = -n
			d = -np.dot(P, n).mean()
			dists = np.abs(np.dot(X, n)+d)
			inliers = dists < max_dist
			n_inliers = inliers.sum()
			if n_inliers > max_n_inliers:
				max_n_inliers = n_inliers
				max_inliers = inliers
				max_plane = np.hstack([n,d])
		#print max_n_inliers/float(N)
		if max_n_inliers/float(N) < inlier_ratio:
			labels[inds] = 0
			continue

		labels[inds] = 0
		inds[np.where(inds)] = max_inliers
		labels[inds] = l2
		_, cont, _ = cv2.findContours(np.uint8(inds*255),
								   cv2.RETR_EXTERNAL,
								   cv2.CHAIN_APPROX_SIMPLE)
		if convex_hull:
			cont = [cv2.convexHull(cont[0])]
		cv2.drawContours(labels, cont, 0, l2, -1)
		planes = np.vstack([planes, max_plane])
		cond = np.isfinite(points).all(2)
		cond = np.logical_and(cond, labels==l2)
		if cond.any():
			center = points[np.where(cond)].mean(0)
			centers = np.vstack([centers, center])
			l2 += 1
		else:
			labels[labels==l2] = 0
	return planes, centers

def refine_planes_eig(points, labels):
	l2 = 1
	for l1 in xrange(1, labels.max()+1):
		inds = labels==l1
		N = inds.sum()
		if N < 500:
			labels[inds] = 0
			continue
		X = points[np.where(inds)]
		#x = points[:,:,0][inds]
		#y = points[:,:,1][inds]
		#z = points[:,:,2][inds]
		#X = np.vstack([x,y,z])
		X_mean = np.mean(X, 1)
		X = X - X_mean.reshape(3,-1)
		A = np.dot(X,X.T)
		try:
			e,V = np.linalg.eigh(A)
		except:
			continue
		i = e.argmin()
		v = V[:,i]
		d = np.dot(X_mean, v)
		if np.sqrt(e[i]/N) > .9 or abs(v[1]) < .95:
			labels[inds] = 0
		else:
			labels[inds] = l2
			l2 += 1

def extract_rectangle(lines, points):
	n = len(lines)
	dirs = np.zeros((n, 3))
	for i, line in enumerate(lines):
		x1,y1,x2,y2 = line
		p1 = points[y1,x1,:]
		p2 = points[y2,x2,:]
		dirs[i,:] = p1-p2
	lengths = np.sqrt(np.sum(dirs*dirs, 1))
	dirs /= lengths.reshape(-1,1)
	cos = np.abs(np.dot(dirs, dirs.T))
	inds = np.argsort(cos.flat)
	pair2 = None
	for i in xrange(n*(n-1)/2):
		pair1 = inds[i*2]
		if cos.flat[pair1] > .1:
			break
		pair1_1 = pair1 % n
		pair1_2 = pair1 / n

		for j in xrange(i+1, n*(n-1)/2):
			pair2 = inds[j*2]
			if cos.flat[pair2] > .1:
				pair2 = None
				break
			pair2_1 = pair2 % n
			pair2_2 = pair2 / n
			if (pair1_1 == pair2_1 and pair1_1 == pair2_2)\
			   or (pair1_2 == pair2_1 and pair1_2 == pair2_2):
				pair2 = None
				continue
			if cos[pair1_1, pair2_1] < .9 and cos[pair1_1, pair2_2] < .9:
				pair2 = None
				continue
			#TODO: further check
		if pair2 is not None:
			break
	if pair2 is None:
		return None
	#TODO: return 4 points

def lines_3d(lines, points):
	n = len(lines)
	dirs = np.zeros((n, 3))
	centers = np.zeros((n, 3))
	for i, line in enumerate(lines):
		x1,y1,x2,y2 = line
		p1 = points[y1,x1,:]
		p2 = points[y2,x2,:]
		dirs[i,:] = p1-p2
		centers[i,:] = (p1+p2)/2
	lengths = np.sqrt(np.sum(dirs*dirs, 1))
	dirs /= lengths.reshape(-1,1)
	return centers, dirs, lengths

# convert transform message to rotation matrix and translation vector
def convert_transform(tfmsg):
	A = quaternion_to_matrix(tfmsg.transform.rotation)
	b = np.array([tfmsg.transform.translation.x,
				  tfmsg.transform.translation.y,
				  tfmsg.transform.translation.z],
				 dtype=np.float32)
	#[vec_in_camera] = np.dot(A, [vec_in_parent]) + b
	return A, b

class DepthLib:
	def __init__(self, tf_buffer=None, points_topic='points'):
		self.gripper_frame = 'hand_palm_link' #TODO: parameter
		if tf_buffer is None:
			self.tf_buffer = tf2_ros.Buffer(rospy.Duration(15.))
			tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		else:
			self.tf_buffer = tf_buffer

                self.points_topic = points_topic
		self.latest_header = None
		self.points = None
		self.rgb = None
		self.normals = None
		self.bridge = CvBridge()

		self.pub = rospy.Publisher('plane_candidates', Image, queue_size=10)

	def set_pointmsg(self, pointmsg):
		self.latest_header = pointmsg.header
		self.points, self.rgb = read_point_cloud(pointmsg)
		normals = -normal_image(self.points)
		smooth_normals(normals)
		self.normals = normals

	def update_pointmsg(self):
		now = rospy.Time.now()
		count = 0
		changed = False
		while not rospy.is_shutdown():
			if self.latest_header and (now - self.latest_header.stamp).to_sec() < 1.5:
				break
			rospy.logwarn('Data is too old.')
			try:
				pointmsg = rospy.wait_for_message(self.points_topic, PointCloud2, .5)
				self.latest_header = pointmsg.header
				changed = True
			except:
				count += 1
				if count > 5:
					rospy.logerr('No message arrived.')
					return
		if changed:
			self.set_pointmsg(pointmsg)

	def calc_normals(self, rect=None):
		if not self.latest_header:
			self.update_pointmsg()
		if rect:
			left, right, top, bottom = map(int, rect)
			left = max(0,left)
			top = max(0,top)
		else:
			left, right, top, bottom = 0, None, 0, None
		return (self.points[top:bottom,left:right,2],
				self.points[top:bottom,left:right],
				self.normals[top:bottom,left:right])

	# get transform to depth frame as rotation matrix and translation vector
	def get_transform(self, parent_frame):
		tf = self.tf_buffer.lookup_transform(parent_frame,
											 self.latest_header.frame_id,
											 self.latest_header.stamp,
											 rospy.Duration(10.))
		return convert_transform(tf)

	def maybe_publish_labels(self, labels):
		if self.pub.get_num_connections() > 0:
			label_normal = cv2.normalize(labels, alpha=0, beta=255, norm_type=cv2.CV_MINMAX)
			label_color = cv2.applyColorMap(np.uint8(label_normal), cv2.COLORMAP_JET)
			self.pub.publish(self.bridge.cv2_to_imgmsg(label_color))

	def getBag(self, req):
		self.update_pointmsg()
		res = GetBagResponse()
		res.object = self.get_bag()
		return res

	def get_bag(self):
		morph_kernel3 = np.array([[0, 0, 1, 0, 0],
								  [0, 0, 1, 0, 0],
								  [1, 1, 1, 1, 1],
								  [0, 0, 1, 0, 0],
								  [0, 0, 1, 0, 0]], np.uint8)
		morph_kernel_vert = np.ones((15,5), np.uint8)

		result = self.calc_normals()
		if result is None:
			return None
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
			return None
		p1 = X[np.where(np.isfinite(X))[0]].mean(0)
		p2 = Y[np.where(np.isfinite(Y))[0]].mean(0)
		if np.linalg.norm(p1-p2) > .15:
			return None

		grasp_pos = p1
		open_dir = p1-p2
		open_dir /= np.linalg.norm(open_dir)
		if open_dir[1] < 0:
			open_dir = -open_dir
		if open_dir[0] < 0:
			open_dir = -open_dir
		aim_dir = [0,0,1]

		obj = Object()
		pose1, pose2 = gripper_pose(grasp_pos, open_dir, aim_dir)
		obj.graspPose.pose = pose2
		obj.graspPose.header.stamp = self.latest_header.stamp
		obj.graspPose.header.frame_id = self.gripper_frame
		obj.approachPose.pose = pose1
		obj.approachPose.header.stamp = self.latest_header.stamp
		obj.approachPose.header.frame_id = self.latest_header.frame_id
		obj.graspWidth = .3
		return obj

	def getPerpendicularPlanes(self, req):
		self.update_pointmsg()
		res = GetPerpendicularPlanesResponse()
		res.planes = self.get_perpendicular_planes(req.axis,
												   req.minimum_points,
												   req.iterations,
												   req.thickness,
												   req.minimum_inlier_ratio)
		return res

	def get_perpendicular_planes(self, axis,
								 minimum_points=1000,
								 iterations=20,
								 thickness=0.03,
								 minimum_inlier_ratio=.6):
		result = self.calc_normals()
		if result is None:
			return GetPerpendicularPlanesResponse()
		depth, points, normals = result
		rot, tr = self.get_transform(axis.header.frame_id)
		plane_norm = [axis.vector.x,
					  axis.vector.y,
					  axis.vector.z]
		plane_norm = np.dot(rot, plane_norm)
		cos = np.dot(normals, plane_norm)
		ret, thresh = cv2.threshold(np.uint8((cos+1)/2*255),
									230, 255, cv2.THRESH_BINARY)
		labels = cluster_planes(depth, thresh)
		#self.maybe_publish_labels(labels)
		params, centers = refine_planes(points, labels,
										min_points=minimum_points,
										n_samples=iterations,
										max_dist=thickness,
										inlier_ratio=minimum_inlier_ratio)
		self.maybe_publish_labels(labels)
		n_planes = labels.max()
		planes = [Plane() for _ in xrange(n_planes)]
		for i in xrange(n_planes):
			planes[i].normal.vector.x = params[i+1,0]
			planes[i].normal.vector.y = params[i+1,1]
			planes[i].normal.vector.z = params[i+1,2]
			planes[i].normal.header.stamp = self.latest_header.stamp
			planes[i].normal.header.frame_id = self.latest_header.frame_id
			planes[i].center.point.x = centers[i+1,0]
			planes[i].center.point.y = centers[i+1,1]
			planes[i].center.point.z = centers[i+1,2]
			planes[i].center.header.stamp = self.latest_header.stamp
			planes[i].center.header.frame_id = self.latest_header.frame_id
		return planes

	def getParallelPlanes(self, req):
		self.update_pointmsg()
		res = GetParallelPlanesResponse()
		res.planes = self.get_parallel_planes(req.axis,
											  req.minimum_points,
											  req.iterations,
											  req.thickness,
											  req.minimum_inlier_ratio)
		return res

	def getParallelPlanes2(self, req):
		self.update_pointmsg()
		res = GetParallelPlanesResponse()
		res.planes = self.get_parallel_planes(req.axis,
											  req.minimum_points,
											  req.iterations,
											  req.thickness,
											  req.minimum_inlier_ratio,
											  opening_kernel_size=70)
		return res

	def get_parallel_planes(self, axis,
							minimum_points=1000,
							iterations=20,
							thickness=0.03,
							minimum_inlier_ratio=0.6,
							angle_allowance=0.3,
							opening_kernel_size=None):
		result = self.calc_normals()
		if result is None:
			return None
		depth, points, normals = result
		rot, tr = self.get_transform(axis.header.frame_id)
		plane_norm = [axis.vector.x,
					  axis.vector.y,
					  axis.vector.z]
		plane_norm = np.dot(rot, plane_norm)
		cos = np.dot(normals, plane_norm)
		sin = np.sqrt(1-cos*cos)
		sin_thresh = np.cos(angle_allowance)
		ret, thresh = cv2.threshold(np.uint8((sin+1)/2*255),
									sin_thresh*255, 255, cv2.THRESH_BINARY)
		if opening_kernel_size is None:
			labels = cluster_planes(depth, thresh)
		else:
			kernel = np.ones((opening_kernel_size, opening_kernel_size),
							 dtype=np.uint8)
			labels = cluster_planes(depth, thresh,
									open_kernel=kernel)
		#self.maybe_publish_labels(labels)
		params, centers = refine_planes(points, labels,
										min_points=minimum_points,
										n_samples=iterations,
										max_dist=thickness,
										inlier_ratio=minimum_inlier_ratio)
		self.maybe_publish_labels(labels)
		n_planes = labels.max()
		planes = [Plane() for _ in xrange(n_planes)]
		for i in xrange(n_planes):
			planes[i].normal.vector.x = params[i+1,0]
			planes[i].normal.vector.y = params[i+1,1]
			planes[i].normal.vector.z = params[i+1,2]
			planes[i].normal.header.stamp = self.latest_header.stamp
			planes[i].normal.header.frame_id = self.latest_header.frame_id
			planes[i].center.point.x = centers[i+1,0]
			planes[i].center.point.y = centers[i+1,1]
			planes[i].center.point.z = centers[i+1,2]
			planes[i].center.header.stamp = self.latest_header.stamp
			planes[i].center.header.frame_id = self.latest_header.frame_id
		return planes

	def getObject(self, req):
		self.update_pointmsg()
		obj = self.get_object((req.left, req.right, req.top, req.bottom))
		return GetObjectResponse(obj)

	def get_object(self, rect):
		left, right, top, bottom = rect
		result = self.calc_normals(rect)
		if result is None:
			return None
		depth, points, normals = result
		rgbimg = self.rgb[max(0,int(top)):int(bottom),
						  max(0,int(left)):int(right), :]
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
			return None
		i = largest_region(labels)
		labels[labels != i] = 0
		self.maybe_publish_labels(labels)
		inliers = labels==i
		x = points[:,:,0][inliers]
		y = points[:,:,1][inliers]
		z = points[:,:,2][inliers]
		obj = Object()
		obj.center.header.stamp = self.latest_header.stamp
		obj.center.header.frame_id = self.latest_header.frame_id
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
		obj.graspPose.header.stamp = self.latest_header.stamp
		obj.graspPose.header.frame_id = self.gripper_frame
		obj.approachPose.pose = pose1
		obj.approachPose.header.stamp = self.latest_header.stamp
		obj.approachPose.header.frame_id = self.latest_header.frame_id
		obj.graspWidth = width
		mask = np.uint8(inliers*255)
		obj.colorHistogram = cv2.calcHist([rgbimg], [0], mask, [180], [0,180]).reshape(-1)
		return obj

	def getTableTopObject(self, req):
		self.update_pointmsg()
		obj = self.get_table_top_object((req.left, req.right, req.top, req.bottom),
										req.table_height)
		return GetTableTopObjectResponse(obj)

	def get_table_top_object(self, rect, table_height):
		left, right, top, bottom = rect
		height = table_height
		result = self.calc_normals(rect)
		if result is None:
			return None
		depth, points, normals = result
		rgbimg = self.rgb[max(0,int(top)):int(bottom),
						  max(0,int(left)):int(right), :]
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
			return None
		i = largest_region(labels)
		labels[labels != i] = 0
		self.maybe_publish_labels(labels)
		inliers = labels==i
		x = points[:,:,0][inliers]
		y = points[:,:,1][inliers]
		z = points[:,:,2][inliers]
		obj = Object()
		obj.center.header.stamp = self.latest_header.stamp
		obj.center.header.frame_id = self.latest_header.frame_id
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
		back_dist = 0.5
		try:
			grasp_pos_base = self.tf_buffer.transform(obj.center, 'base_footprint', rospy.Duration(3.))
			rospy.loginfo('grasp_pos = {}'.format(grasp_pos_base))
			back_dist = grasp_pos_base.point.x - 0.2
		except:
			rospy.logerr(traceback.format_exc())
			rospy.logwarn('Failed to transform. Using back_dist=0.5.')
		rospy.loginfo('back_dist = {}'.format(back_dist))
		pose1, pose2 = gripper_pose(grasp_pos, open_dir, aim_dir, back_dist=back_dist)
		obj.graspPose.pose = pose2
		obj.graspPose.header.stamp = self.latest_header.stamp
		obj.graspPose.header.frame_id = self.gripper_frame
		obj.approachPose.pose = pose1
		obj.approachPose.header.stamp = self.latest_header.stamp
		obj.approachPose.header.frame_id = self.latest_header.frame_id
		obj.graspWidth = width
		mask = np.uint8(inliers*255)
		obj.colorHistogram = cv2.calcHist([rgbimg], [0], mask, [180], [0,180]).reshape(-1)
		return obj

	def getTray(self, req):
		self.update_pointmsg()
		obj = self.get_tray()
		return GetTrayResponse(obj)

	def get_tray(self):
		result = self.calc_normals()
		if result is None:
			return None
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
			return None
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
			return None
		'''
		rect = extract_rectangle(lines[0], points)
		if rect is None:
			return None
		for i, p in enumerate(rect):
			res.endPoints[i].point.x = p[0]
			res.endPoints[i].point.y = p[1]
			res.endPoints[i].point.z = p[2]
			res.endPoints[i].header.frame_id = self.latest_header.frame_id
			res.endPoints[i].header.stamp = self.latest_header.stamp
		'''
		centers, dirs, lengths = lines_3d(lines[0], points)
		horiz = np.dot(dirs, rot[:,1])**2 > .5
		if horiz.sum() == 0:
			return None
		centers = centers[np.where(horiz)]
		dirs = dirs[np.where(horiz)]
		i = centers[:,2].argmin()

		obj = Object()
		open_dir = np.cross(norm, dirs[i])
		if np.dot(open_dir , rot[:,2]) < -0.5\
		   or np.dot(open_dir, rot[:,1]) > 0.5: # if sucker up or right
			open_dir = -open_dir
		pose1, pose2 = gripper_pose(centers[i], open_dir, -norm)
		obj.graspPose.pose = pose2
		obj.graspPose.header.frame_id = self.gripper_frame
		obj.graspPose.header.stamp = self.latest_header.stamp
		obj.approachPose.pose = pose1
		obj.approachPose.header.frame_id = self.latest_header.frame_id
		obj.approachPose.header.stamp = self.latest_header.stamp
		return obj
