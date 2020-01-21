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

def pos(x):
	if x < 0:
		x = 0
	return x

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

		self.slots = {
					'101B1':[3.768, 0.0, 0, 1.2575, 1.23, 0.4, 0, 0],
					'101B2':[3.768, 0.0, 1.23, 1.2575, 1.23, 0.4, 0,  0],
					'B03F3B':[3.768, 0.0, 2.46, 1.2575, 0.94  , 0.4, 0, 0],
					'101A1':[2.515, 0.0, 0, 1.2575, 1.23, 0.4, 0, 0],
					'101A2':[2.515, 0.0, 1.23, 1.2575, 1.23, 0.4, 0, 0],
					'101A3':[2.515, 0.0, 2.46, 1.2575, 0.94  , 0.4, 0, 0],
					'100B1':[1.2575, 0.0, 0, 1.2575, 1.23, 0.4, 0, 0],
					'100B2':[1.2575, 0.0, 1.23, 1.2575, 1.23, 0.4, 0, 0],
					'100B3':[1.2575, 0.0, 2.46, 1.2575, 0.94  , 0.4, 0, 0],
					'100A1':[0, 0.0, 0, 1.2575, 1.23, 0.4, 0, 0],
					'100A2':[0, 0.0, 1.23, 1.2575, 1.23, 0.4, 0, 0],
					'100A3':[0, 0.0, 2.46, 1.2575, 0.94  , 0.4, 0, 0]
					}
		#self.publish_slot(self.slots)

		self.den = 22.0    #16.0 for 0.05 resolution; 4.0 for 0.1 resolution
		self.bias = 200.0
		self.bias1 = 200.0


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
		# Consider north flight 
		for cell in xyz:
			for key, value in self.slots.items():
				if (value[0]<=cell[0]<=value[0]+value[3]) and (value[1]<=cell[1]<=value[1]+value[5]) and (value[2]<=cell[2]<=value[2]+value[4]):
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

		
		print "\n      100A_    100B_    101A_    101B_"
		print("counts/slot:")
		#print "%8.2f %8.2f %8.2f %8.2f " % (self.slots["C2003"][7]/self.slots["C2003"][6],self.slots["C2002"][7]/self.slots["C2002"][6], self.slots["C2001"][7]/self.slots["C2001"][6], self.slots["C2000"][7]/self.slots["C2000"][6])
		print " %s %8d %8d %8d %8d" % ("3", self.slots["100A3"][6], self.slots["100B3"][6], self.slots["101A3"][6], self.slots["B03F3B"][6])
		print " %s %8d %8d %8d %8d" % ("2", self.slots["100A2"][6], self.slots["100B2"][6], self.slots["101A2"][6], self.slots["101B2"][6])
		print " %s %8d %8d %8d %8d" % ("1", self.slots["100A1"][6], self.slots["100B1"][6], self.slots["101A1"][6], self.slots["101B1"][6])
		print "\npercent(%):"
		print " %s %8.1f %8.1f %8.1f %8.1f" % ("3", pos((self.slots["100A3"][6]-self.bias)/self.den*1.33), pos((self.slots["100B3"][6]-self.bias)/self.den*1.33), pos((self.slots["101A3"][6]-self.bias)/self.den*1.33), pos((self.slots["B03F3B"][6]-self.bias)/self.den*1.33))
		print " %s %8.1f %8.1f %8.1f %8.1f" % ("2", pos((self.slots["100A2"][6]-self.bias)/self.den), pos((self.slots["100B2"][6]-self.bias)/self.den), pos((self.slots["101A2"][6]-self.bias)/self.den), pos((self.slots["101B2"][6]-self.bias)/self.den))
		print " %s %8.1f %8.1f %8.1f %8.1f" % ("1", pos((self.slots["100A1"][6]-self.bias1)/self.den), pos((self.slots["100B1"][6]-self.bias1)/self.den), pos((self.slots["101A1"][6]-self.bias1)/self.den), pos((self.slots["101B1"][6]-self.bias1)/self.den))
		


		#print "   %8.2f %8.2f %8.2f %8.2f " % ( self.slots["C2003"][7]/self.slots["C2003"][6],self.slots["C2002"][7]/self.slots["C2002"][6], self.slots["C2001"][7]/self.slots["C2001"][6], self.slots["C2000"][7]/self.slots["C2000"][6])
		#print  self.slots["C2003"][7]
		#print "HI  %8.2f" % (5.1234)

	def publish_slot(self, slots):
		
		i = 0
		markerArray = MarkerArray()

		# South flight only for now
		for key,value in slots.items():

			marker = Marker()
			marker.header.frame_id = "map"
			marker.type = Marker.CUBE
			marker.action = Marker.ADD
			marker.id = i
			marker.pose.position.x = value[0] + value[3]/2.   # for south flight '-'
			marker.pose.position.y = value[1] + value[5]/2.   # for south flight '-'
			marker.pose.position.z = value[2] + value[4]/2.
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
	rospy.init_node('check_slot_node', anonymous=True)
	
	CheckSlots()

	rospy.spin()
