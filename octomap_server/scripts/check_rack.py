#!/usr/bin/env python
from scipy.spatial import KDTree, distance
import rospy
import ros_numpy
import numpy as np
from std_msgs.msg import Bool, String
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandTOL
import tf2_ros
import tf2_geometry_msgs
from std_srvs.srv import Empty
import math as m
from visualization_msgs.msg import Marker, MarkerArray
import pincair_msgs.msg

class CheckSlots:

	def __init__(self):
			# To visualize each slot
		self.slot_pub = rospy.Publisher('/slots', MarkerArray,queue_size=10)
		
		# Initialize tf listeners
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		# To subscribe key rostopics
		self.octomap_pcl_sub = rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, self.octomap_pcl_callback, queue_size = 1)
		self.fcu_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.fcu_pose_callback)

		self.slots = {'A2005':[8.16, -3, 0, 1.2575, 1.23, 0.92, 0],
					'B2005':[8.16, -3, 1.23, 1.2575, 1.23, 0.92, 0],
					'C2005':[8.16, -3, 2.46, 1.2575, 0.74, 0.92, 0],
					'A2004':[6.9025, -3, 0, 1.2575, 1.23, 0.92, 0],
					'B2004':[6.9025, -3, 1.23, 1.2575, 1.23, 0.92, 0],
					'C2004':[6.9025, -3, 2.46, 1.2575, 0.74, 0.92, 0],
					'A2003':[5.645, -3, 0, 1.2575, 1.23, 0.92, 0],
					'B2003':[5.645, -3, 1.23, 1.2575, 1.23, 0.92, 0],
					'C2003':[5.645, -3, 2.46, 1.2575, 0.74, 0.92, 0],
					'A2002':[4.08, -3, 0, 1.2575, 1.23, 0.92, 0],
					'B2002':[4.08, -3, 1.23, 1.2575, 1.23, 0.92, 0],
					'C2002':[4.08, -3, 2.46, 1.2575, 0.74, 0.92, 0],
					'A2001':[2.515, -3, 0, 1.2575, 1.23, 0.92, 0],
					'B2001':[2.515, -3, 1.23, 1.2575, 1.23, 0.92, 0],
					'C2001':[2.515, -3, 2.46, 1.2575, 0.74, 0.92, 0],
					'A2000':[1.2575, -3, 0, 1.2575, 1.23, 0.92, 0],
					'B2000':[1.2575, -3, 1.23, 1.2575, 1.23, 0.92, 0],
					'C2000':[1.2575, -3, 2.46, 1.2575, 0.74, 0.92, 0]}
	
	def fcu_pose_callback(self, msg):
		self.fcu_pose = msg
		return
	
	def octomap_pcl_callback(self, msg):
		try:
			# Convert Pointcloud2 into a list
			xyz = ros_numpy.numpify(msg).tolist()
			tree = KDTree(xyz)
		except:
			rospy.loginfo('Check_slots_node: bad incoming data in octomap_call_callback')
			return

		# octomap dat is in map frame. fcu_pose is w.r.t to local_origin

		# Get fcu w.r.t. map
		map_to_local_origin = self.tf_buffer.lookup_transform("map", self.fcu_pose.header.frame_id, rospy.Time())

		# Get fcu_pose w.r.t map frame
		fcu_pose_map = tf2_geometry_msgs.do_transform_pose(self.fcu_pose, map_to_local_origin)
		print 'pcl size: ',len(xyz)
		# print  'xyz: ',xyz[0][0]

		for key, value in self.slots.items():
			self.slots[key][6] = 0

		for pcl in xyz:
			for key, value in self.slots.items():
				if (value[0]<=pcl[0]<=value[0]+value[3]) and (value[1]<=pcl[1]<=value[1]+value[4]) and (value[2]<=pcl[2]<=value[2]+value[5]):
						self.slots[key][6] += 1
		
		print "\nDrone Pose in Map: \n", fcu_pose_map.pose.position
		
		for key, value in self.slots.items():
			print key+" counts: ", value[6]
		return
		
if __name__ == '__main__':
	print("Starting Slot Counts .......")
	rospy.init_node('check_slot_node', anonymous=False)
	CheckSlots()
	rospy.spin()
