#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image, PointCloud2
from person_locator.srv import *

from cv_bridge import CvBridge, CvBridgeError
import cv2

import os
from subprocess import call
from rospkg import RosPack

# Context Manager for handling directories for subprocess call
class cd:
    """Context manager for changing the current working directory"""
    def __init__(self, newPath):
        self.newPath = os.path.expanduser(newPath)

    def __enter__(self):
        self.savedPath = os.getcwd()
        os.chdir(self.newPath)

    def __exit__(self, etype, value, traceback):
        os.chdir(self.savedPath)


# Some initial stuff
rospack = RosPack()
face_detect_dir = rospack.get_path('person_locator') +'/scripts/face_detector/'

bridge = CvBridge()

class PersonLocator():
	def __init__(self):
		self.rgb_img = Image()
		self.update_frames()

	def get_face(self):
		""" returns the bounding box of the face for the name 
		in the request"""

		self.update_frames()
		# print('Image Time Stamp:' + str(self.rgb_img.header.stamp))
		# print('PointCloud2 Time Stamp:' + str(self.pointcloud.header.stamp))

		self.save_img()

		# open pkg directory, run face detection and find bounding box
		with cd(face_detect_dir):
			call("python3 main.py", shell=True)	
			# read the bounding box from the file below
			with open('/runtime/location.txt', 'r') as fn:
				location_str = fn.readline().strip()

		bounding_box = eval(location_str)
		return bounding_box

	def save_img(self):
		
		try:
			#print(self.rgb_img)
			cv2_img = bridge.imgmsg_to_cv2(self.rgb_img, "bgr8")
		except CvBridgeError, e:
			print(e)
		else:
			with cd(face_detect_dir):
				cv2.imwrite('/runtime/camera_img.jpg', cv2_img)

	def update_rgb_img(self,img):
		#print("update image called")
		self.rgb_img = img
		self.image_found = True

	def update_pointcloud(self,pc):
		#print("update pointcloud called")
		self.pointcloud = pc
	
	def update_frames(self):
		""" capture the color image for face recognition and 
		point cloud for determining location"""
		rospy.Subscriber("head_camera/rgb/image_raw", Image, self.update_rgb_img)
		rospy.Subscriber("head_camera/depth_registered/points", PointCloud2, self.update_pointcloud)

	def get_position(self, req):
		print('Finding location of '+ str(req.person_name)+ '...')
		
		# print('finding ' + req)
		bounding_box = self.get_face()

		# TODO: Use bounding box to find the xy position of the person in map frame
		# Do this with image_geometry package
		position = GetPersonPositionResponse()

		return position


# def main_run():
# 	person_locator_srv = PersonLocator()
# 	rospy.init_node('person_locator_server', anonymous=True)
# 	rospy.loginfo('Starting person_locator service ... ')
# 	person_locator_srv.update_frames()
# 	while(not person_locator_srv.image_found):
# 		sleep(.5)
# 	person_locator_srv.save_img()
	
# 	#rospy.Service('person_locator', GetPersonPosition, person_locator_srv.get_position)

# 	location = person_locator_srv.get_position('name')
# 	print(location)

# 	rospy.spin()


if __name__ == '__main__':
	# main_run()
	
	person_locator_srv = PersonLocator()
	
	rospy.init_node('person_locator_server', anonymous=True)
	rospy.loginfo('Starting person_locator service ... ')
	
	# Start service 
	rospy.Service('person_locator', GetPersonPosition, person_locator_srv.get_position)

	# location = person_locator_srv.get_position('name')

	# Keep node from exiting
	rospy.spin()
