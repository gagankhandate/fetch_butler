#!/usr/bin/env python

import rospy
from subprocess import call
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
from time import sleep

from person_locator.srv import *

bridge = CvBridge()

LOCATION_FILENAME = 'location.txt'

class PersonLocator():
	def __init__(self):
		self.rgb_img = Image()
		self.pointcloud = PointCloud2()
		self.image_needed = True
		self.image_found = False

	def get_face(self):
		""" returns the bounding box of the face for the name 
		in the request"""
		call("python3 robot_face_detector/main.py")
		

	def save_img(self):
		try:
			print(self.rgb_img)
			cv2_img = bridge.imgmsg_to_cv2(self.rgb_img, "bgr8")
		except CvBridgeError, e:
			print(e)
		else:
			cv2.imwrite('images/camera_img.jpg', cv2_img)

	def update_rgb_img(self,img):
		print("update image called")
		self.rgb_img = img
		self.image_found = True

	def update_pointcloud(self,pc):
		print("update pointcloud called")
		self.pointcloud = pc
	
	def update_frames(self):
		""" capture the color image for face recognition and 
		point cloud for determining location"""
		rospy.Subscriber("head_camera/rgb/image_raw", Image, self.update_rgb_img)
		rospy.Subscriber("head_camera/depth_registered/points", PointCloud2, self.update_pointcloud)

	def get_position(self, req):
		#print('Finding location of '+ str(req.person_name)+ '...')
		print('finding ' + req)
		self.update_frames()

		# print(self.rgb_img)
		# print(self.pointcloud)

		with open(LOCATION_FILENAME, 'r') as fn:
			location_str = fn.readline().strip()
		#print(location_str)

		bouding_box = eval(location_str)

		print('Image Time Stamp:' + str(self.rgb_img.header.stamp))
		print('PointCloud2 Time Stamp:' + str(self.pointcloud.header.stamp))

		return GetPersonPositionResponse()


def main_run():
	person_locator_srv = PersonLocator()
	rospy.init_node('person_locator_server', anonymous=True)
	rospy.loginfo('Starting person_locator service ... ')
	person_locator_srv.update_frames()
	#while(not person_locator_srv.image_found):
	#	sleep(.5)
	#person_locator_srv.save_img()
	
	#rospy.Service('person_locator', GetPersonPosition, person_locator_srv.get_position)

	location = person_locator_srv.get_position('name')
	print(location)

	rospy.spin()


if __name__ == '__main__':
	main_run()
