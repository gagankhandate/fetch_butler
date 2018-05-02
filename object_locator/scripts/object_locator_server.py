#!/usr/bin/env python
import rospy
from obj_recognition.msg import SegmentedClustersArray
#from object_locator.srv import GetObjectPoseRequest, GetObjectPoseReply
from object_locator.srv import *
import sensor_msgs.point_cloud2
import time
from geometry_msgs.msg import Point, Quaternion

class ObjectLocator(object):

	def __init__(self):
		self.seg_object_pc = None

	def pc_cb(self, pc_array):
		#print('object locator point cloud callback called')
		self.seg_object_pc = pc_array
		#rospy.loginfo('Recieved segmented object pointclouds')
		#print(type(pc_array.clusters))
		#print(type(pc_array.clusters[0]))
		#print(pc_array.clusters[0].fields[0], pc_array.clusters[0].fields[1], pc_array.clusters[0].fields[2])

	def update_pc(self):
		""" """
		print('obj locator update pc called')
		rospy.Subscriber("/obj_recognition/pcl_clusters", SegmentedClustersArray, self.pc_cb)

	def get_pos(self, req):
		if(type(req) == str):
			print('getting' + req)
		else:
			rospy.loginfo('getting object ' + str(req.object_name))
		self.update_pc()
		#print(self.seg_object_pc)
		while self.seg_object_pc == None:
			time.sleep(.3)

		x_sum = 0.0
		x_num = 0.0
		y_sum = 0.0
		y_num = 0.0
		z_sum = 0.0
		z_num = 0.0
		close_point = (100.0,100.0,100.0)
		min_obj_distance = close_point
		for cluster in self.seg_object_pc.clusters:
			for point in sensor_msgs.point_cloud2.read_points(cluster,skip_nans=True):
				#print(point)
				x_sum += point[0]
				x_num += 1.0
				y_sum += point[1]
				y_num += 1.0
				z_sum += point[2]
				z_num += 1.0

			center = (x_sum/x_num,y_sum/y_num,z_sum/z_num)
			print(center)
			if((abs(center[0]) + abs(center[1]) + abs(center[2])) < (abs(min_obj_distance[0]) + abs(min_obj_distance[1])+abs(min_obj_distance[2]))):
				min_obj_distance = center
		
		pos_msg = GetObjectPoseResponse()
		pos_msg.g_pos.position = Point(center[0]-0.02,center[1]-0.22,center[2])
		pos_msg.g_pos.orientation = Quaternion(0.0,0.0,0.0,1.0)
		# return (center, (0.0,0.0,0.0,1.0))
		return pos_msg

if __name__ == '__main__':

	object_locator_srv = ObjectLocator()
	rospy.init_node('object_locator_service', anonymous=True)
	rospy.loginfo('starting object locator service...')

	rospy.Service('object_locator', GetObjectPose, object_locator_srv.get_pos) 
	rospy.sleep(2);
	#object_locator_srv.update_pc()
	print(object_locator_srv.get_pos('pringles'))
	rospy.spin()	
