#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image, PointCloud2
from person_locator.srv import *

class PersonLocator():
	def __init__(self):
		self.rgb_img = Image()
		self.pointcloud = PointCloud2()

	def get_face(self):
		""" returns the bounding box of the face for the name 
		in the request"""

	def update_rgb_img(self,img):
		self.rgb_img = img

	def update_pointcloud(self,pc):
		self.pointcloud = pc
	
	def update_frames(self):
		""" capture the color image for face recognition and 
		point cloud for determining location"""
		rospy.Subscriber("head_camera/rgb/image_raw", Image, self.update_rgb_img)
		rospy.Subscriber("head_camera/depth_registered/points", PointCloud2, self.update_pointcloud)

	def get_position(self, req):
		print('Finding location of '+ str(req.person_name)+ '...')
		self.update_frames()

		# print(self.rgb_img)
		# print(self.pointcloud)

		print('Image Time Stamp:' + str(self.rgb_img.header.stamp))
		print('PointCloud2 Time Stamp:' + str(self.pointcloud.header.stamp))

		return GetPersonPositionResponse()


person_locator_srv = PersonLocator()

if __name__ == '__main__':
	rospy.init_node('person_locator_server', anonymous=True)
	rospy.loginfo('Starting person_locator service ... ')
	rospy.Service('person_locator', GetPersonPosition, person_locator_srv.get_position)

	rospy.spin()




