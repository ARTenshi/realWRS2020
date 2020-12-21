#!/usr/bin/env python

import rospy
from depth_lib.srv import GetPerpendicularPlanes, GetParallelPlanes, GetObject, GetTableTopObject, GetTray, GetBag
from depth_lib import DepthLib

if __name__ == '__main__':
	rospy.init_node('depth_lib')
	lib = DepthLib()
	rospy.Service('get_perpendicular_planes', GetPerpendicularPlanes, lib.getPerpendicularPlanes)
	rospy.Service('get_parallel_planes', GetParallelPlanes, lib.getParallelPlanes)
	rospy.Service('get_parallel_planes2', GetParallelPlanes, lib.getParallelPlanes2)
	rospy.Service('get_object', GetObject, lib.getObject)
	rospy.Service('get_table_top_object', GetTableTopObject, lib.getTableTopObject)
	rospy.Service('get_tray', GetTray, lib.getTray)
	#rospy.Service('get_dish', GetDish, lib.getDish)
	rospy.Service('get_bag', GetBag, lib.getBag)
	rospy.spin()
