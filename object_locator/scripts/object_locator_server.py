#!/usr/bin/env python
import rospy
from obj_recognition.msg import SegmentedClustersArray

class ObjectLocator():

	def __init__():
		self.seg_object_pc = None

	def pc_cb(self, pc_array):
		self.seg_object_pc = pc_array
		rospy.loginfo('Recieved segmented object pointclouds')

	def update_pc():
		""" """
		rospy.Subscriber("/obj_recognition/pcl_clusters", SegmentedClustersArray, self.pc_cb)

if __name__ == '__main__':

	object_locator_srv = ObjectLocator()
	rospy.init_node('object_locator_service', anonymous=True)
	rospy.sleep(5);
	object_locator_srv.update_pc()
	rospy.spin()	