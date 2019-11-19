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

		self.slots = {'A2005':[8.16, -3, 0, 1.2575, 1.23, 0.2, 0, 0],
					'B2005':[8.16, -3, 1.23, 1.2575, 1.23, 0.2, 0, 0],
					'C2005':[8.16, -3, 2.46, 1.2575, 0.74  , 0.2, 0, 0],
					'A2004':[6.9025, -3, 0, 1.2575, 1.23, 0.2, 0, 0],
					'B2004':[6.9025, -3, 1.23, 1.2575, 1.23, 0.2, 0, 0],
					'C2004':[6.9025, -3, 2.46, 1.2575, 0.74  , 0.2, 0, 0],
					'A2003':[5.645, -3, 0, 1.565, 1.23, 0.2, 0, 0],
					'B2003':[5.645, -3, 1.23, 1.565, 1.23, 0.2, 0, 0],
					'C2003':[5.645, -3, 2.46, 1.565, 0.74  , 0.2, 0, 0],
					'A2002':[4.08, -3, 0, 1.5655, 1.23, 0.2, 0, 0],
					'B2002':[4.08, -3, 1.23, 1.565, 1.23, 0.2, 0,  0],
					'C2002':[4.08, -3, 2.46, 1.565, 0.74  , 0.2, 0, 0],
					'A2001':[2.515, -3, 0, 1.2575, 1.23, 0.2, 0, 0],
					'B2001':[2.515, -3, 1.23, 1.2575, 1.23, 0.2, 0, 0],
					'C2001':[2.515, -3, 2.46, 1.2575, 0.74  , 0.2, 0, 0],
					'A2000':[1.2575, -3, 0, 1.2575, 1.23, 0.2, 0, 0],
					'B2000':[1.2575, -3, 1.23, 1.2575, 1.23, 0.2, 0, 0],
					'C2000':[1.2575, -3, 2.46, 1.2575, 0.74  , 0.2, 0, 0]}
		#self.publish_slot(self.slots)

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

			
		print "\nDrone position in Map frame: \n", fcu_pose_map.pose.position
		
		print 'Total number of cells : ',len(xyz)
		# print  'xyz: ',xyz[0][0]

		# Intialize occupied cell counts and average depth
		for key in self.slots:
			self.slots[key][6] = 0
			self.slots[key][7] = 0.

		# Check if each cell is within the designated slots and then add counts and depth
		# Consider south flight only for now
		for cell in xyz:
			for key, value in self.slots.items():
				if (value[0]-value[3]<=cell[0]<=value[0]) and (value[1]-value[5]<=cell[1]<=value[1]) and (value[2]<=cell[2]<=value[2]+value[4]):
						value[6] += 1
						value[7] += cell[1]
		
		# Print the number of cells in the designated slot and its average depth of
		"""for key, value in self.slots.items():
			
			if value[6] != 0.:
				print key+" count: ", value[6], "  avg depth: ",value[7]/value[6]
			else:
				print key+" count: ", value[6]
		"""
		self.publish_slot(self.slots)


		print "\n   %8d %8d %8d %8d %8d %8d" % (2005, 2004, 2003, 2002, 2001, 2000)
		
		#print "%8.2f %8.2f %8.2f %8.2f " % (self.slots["C2003"][7]/self.slots["C2003"][6],self.slots["C2002"][7]/self.slots["C2002"][6], self.slots["C2001"][7]/self.slots["C2001"][6], self.slots["C2000"][7]/self.slots["C2000"][6])
		print " %s %8d %8d %8d %8d %8d %8d" % ("C", self.slots["C2005"][6], self.slots["C2004"][6], self.slots["C2003"][6], self.slots["C2002"][6], self.slots["C2001"][6], self.slots["C2000"][6])
		print " %s %8d %8d %8d %8d %8d %8d" % ("B", self.slots["B2005"][6], self.slots["B2004"][6], self.slots["B2003"][6], self.slots["B2002"][6], self.slots["B2001"][6], self.slots["B2000"][6])
		print " %s %8d %8d %8d %8d %8d %8d" % ("A", self.slots["A2005"][6], self.slots["A2004"][6], self.slots["A2003"][6], self.slots["A2002"][6], self.slots["A2001"][6], self.slots["A2000"][6])
		


		#print "   %8.2f %8.2f %8.2f %8.2f " % ( self.slots["C2003"][7]/self.slots["C2003"][6],self.slots["C2002"][7]/self.slots["C2002"][6], self.slots["C2001"][7]/self.slots["C2001"][6], self.slots["C2000"][7]/self.slots["C2000"][6])
		#print  self.slots["C2003"][7]
		#print "HI  %8.2f" % (5.1234)

	def publish_slot(self, slots):
		
		i = 0
		markerArray = MarkerArray()
		for key,value in slots.items():

			marker = Marker()
			marker.header.frame_id = "map"
			marker.type = Marker.CUBE
			marker.action = Marker.ADD
			marker.id = i
			marker.pose.position.x = value[0] - value[3]/2.;
			marker.pose.position.y = value[1] + value[5]/2
			marker.pose.position.z = value[2] + value[4]/2
			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0
			marker.scale.x = value[3]
			marker.scale.y = value[5]    # depth
			marker.scale.z = value[4]    # height
			marker.color.a = 0.5
			marker.color.r = 0.5
			marker.color.g = 0.5
			marker.color.b = 0.5
			markerArray.markers.append(marker)
			#marker.liftime = ros.Duration()
			i += 1
		self.slot_pub.publish(markerArray)	


if __name__ == '__main__':
	print("Computing occupied cells and average depth.......")
	rospy.init_node('check_slot_node', anonymous=False)
	
	CheckSlots()

	rospy.spin()
