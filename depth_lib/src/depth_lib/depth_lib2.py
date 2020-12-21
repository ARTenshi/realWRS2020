import rospy
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3Stamped, Pose, Quaternion
import tf2_ros
import tf2_geometry_msgs
import traceback

from depth_lib.msg import Object, Plane

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
    m11 = 1.0 - 2.0 * q.y * q.y - 2.0 * q.z * q.z;
    m12 = 2.0 * q.x * q.y + 2.0 * q.w * q.z;
    m13 = 2.0 * q.x * q.z - 2.0 * q.w * q.y;
    m21 = 2.0 * q.x * q.y - 2.0 * q.w * q.z;
    m22 = 1.0 - 2.0 * q.x * q.x - 2.0 * q.z * q.z;
    m23 = 2.0 * q.y * q.z + 2.0 * q.w * q.x;
    m31 = 2.0 * q.x * q.z + 2.0 * q.w * q.y;
    m32 = 2.0 * q.y * q.z - 2.0 * q.w * q.x;
    m33 = 1.0 - 2.0 * q.x * q.x - 2.0 * q.y * q.y;
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

def depth_to_points(depth_img, P):
    X,Y = np.meshgrid(xrange(depth_img.shape[1]), xrange(depth_img.shape[0]))
    points = np.array(np.dot(P, np.vstack([X.flat,Y.flat,np.ones_like(X.flat)])))
    points *= depth_img.reshape(1,-1)
    points = np.reshape(points, (-1,)+depth_img.shape)
    points = np.transpose(points, (1,2,0))
    return points

def point_to_pixel(point, P_inv):
    p = point/point[2]
    q = np.array(np.dot(P_inv, p))
    return q[0], q[1]

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
    depth8 = np.uint8([])
    if len(depth) > 0:
        depth8 = np.uint8(depth/depth[np.isfinite(depth)].max()*255)
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
    def __init__(self, tf_buffer=None):
        self.gripper_frame = 'hand_palm_link' #TODO: parameter
        if tf_buffer is None:
            self.tf_buffer = tf2_ros.Buffer(rospy.Duration(15.))
            tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        else:
            self.tf_buffer = tf_buffer

        self.points = None
        self.rgb = None
        self.normals = None
        self.depth = None
        self.header = None

    def set_pointmsg(self, pointmsg):
        self.header = pointmsg.header
        self.points, self.rgb = read_point_cloud(pointmsg)
        self.depth = self.points[:,:,2]
        normals = -normal_image(self.points)
        smooth_normals(normals)
        self.normals = normals

    # get transform to depth frame as rotation matrix and translation vector
    def get_transform(self, parent_frame):
        tf = self.tf_buffer.lookup_transform(parent_frame,
                                             self.header.frame_id,
                                             self.header.stamp,
                                             rospy.Duration(30.))
        return convert_transform(tf)

    def get_perpendicular_planes(self, axis,
                                 minimum_points=1000,
                                 iterations=20,
                                 thickness=0.03,
                                 minimum_inlier_ratio=.6):
        depth, points, normals = self.depth, self.points, self.normals
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

    def get_parallel_planes(self, axis,
                            minimum_points=1000,
                            iterations=20,
                            thickness=0.03,
                            minimum_inlier_ratio=0.6,
                            angle_allowance=0.3,
                            opening_kernel_size=None):
        depth, points, normals = self.depth, self.points, self.normals
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

    def get_object(self, left, top, right, bottom):
        top = max(0, int(top))
        left = max(0, int(left))
        bottom = int(bottom)
        right = int(right)
        depth = self.depth[top:bottom,left:right]
        points = self.points[top:bottom,left:right,:]
        normals = self.normals[top:bottom,left:right,:]
        rgb = self.rgb[top:bottom,left:right,:]
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
        inliers = labels==i
        xyz = points[np.where(inliers)]
        #x = points[:,:,0][inliers]
        #y = points[:,:,1][inliers]
        #z = points[:,:,2][inliers]
        obj = Object()
        obj.center.header.stamp = self.header.stamp
        obj.center.header.frame_id = self.header.frame_id
        #obj.center.point.x = x.mean()
        #obj.center.point.y = y.mean()
        #obj.center.point.z = z.mean()
        obj.center.point.x, obj.center.point.y, obj.center.point.z = xyz.mean(0)
	#if cv2.getVersionMajor() in [2, 4]:
	#	cont, _ = cv2.findContours(np.uint8(inliers*255),
	#	                           cv2.RETR_EXTERNAL,
	#	                           cv2.CHAIN_APPROX_SIMPLE)
	#else:
	#	_, cont, _ = cv2.findContours(np.uint8(inliers*255),
	#	                           cv2.RETR_EXTERNAL,
	#	                           cv2.CHAIN_APPROX_SIMPLE)

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
        obj.graspPose.header.stamp = self.header.stamp
        obj.graspPose.header.frame_id = self.gripper_frame
        obj.approachPose.pose = pose1
        obj.approachPose.header.stamp = self.header.stamp
        obj.approachPose.header.frame_id = self.header.frame_id
        obj.graspWidth = width
        mask = np.uint8(inliers*255)
        obj.colorHistogram = cv2.calcHist([rgb], [0], mask, [180], [0,180]).reshape(-1)
        return obj

    def get_table_top_object(self, left, top, right, bottom, table_height):
        top = max(0, int(top))
        left = max(0, int(left))
        bottom = int(bottom)
        right = int(right)
        depth = self.depth[top:bottom,left:right]
        points = self.points[top:bottom,left:right,:]
        normals = self.normals[top:bottom,left:right,:]
        rgb = self.rgb[top:bottom,left:right,:]

        height = table_height
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
        inliers = labels==i
        #x = points[:,:,0][inliers]
        #y = points[:,:,1][inliers]
        #z = points[:,:,2][inliers]
        xyz = points[np.where(inliers)]
        obj = Object()
        obj.center.header.stamp = self.header.stamp
        obj.center.header.frame_id = self.header.frame_id
        obj.center.point.x, obj.center.point.y, obj.center.point.z = xyz.mean(0)
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
        obj.graspPose.header.stamp = self.header.stamp
        obj.graspPose.header.frame_id = self.gripper_frame
        obj.approachPose.pose = pose1
        obj.approachPose.header.stamp = self.header.stamp
        obj.approachPose.header.frame_id = self.header.frame_id
        obj.graspWidth = width
        mask = np.uint8(inliers*255)
        obj.colorHistogram = cv2.calcHist([rgb], [0], mask, [180], [0,180]).reshape(-1)
        return obj
