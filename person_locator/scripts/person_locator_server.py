#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image, PointCloud2
import tf
from person_locator.srv import *

from cv_bridge import CvBridge, CvBridgeError
import cv2

import os
from subprocess import call
from rospkg import RosPack

import struct
import numpy as np

from fetch_face_detector import face_detector


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
face_detect_dir = rospack.get_path('person_locator') +'/scripts/'

bridge = CvBridge()

class PersonLocator():
	def __init__(self):
		print('init started')
		self.headcam_rgb = []
		self.headcam_pc = []
		self.update_frames()
		self.tf_listener = tf.TransformListener()
		self.my_face_detector = face_detector()

	def update_rgb(self,img):
		#print("update image called")
		self.headcam_rgb = img

	def update_pc(self,pc):
		#print("update pointcloud called")
		self.headcam_pc = pc

	def update_frames(self):
		""" capture the color image for face recognition and 
		point cloud for determining location"""
		rospy.Subscriber("head_camera/rgb/image_raw", Image, self.update_rgb)
		rospy.Subscriber("head_camera/depth_registered/points", PointCloud2, self.update_pc) 

	def get_face(self):
		""" returns the bounding box of the face for the name 
		in the request"""
		print('updating frames')
		self.update_frames()
		print('saving image')
		#self.save_img()

		print('getting face')
		# open pkg directory, run face detection and find bounding box
		# with cd(face_detect_dir):
		# 	call("python3 main.py", shell=True)	
		# 	# read the bounding box from the file below
		# 	with open('location.txt', 'r') as fn:
		# 		location_str = fn.readline().strip()
		# bounding_box = eval(location_str)
		print('cding to face_detect_dir')		
		with cd(face_detect_dir):
			print('getting bounding box')
			bounding_box = self.my_face_detector.run()
			print('got it')

		return bounding_box

	def save_img(self):
		try:
			#print(self.rgb_img)
			cv2_img = bridge.imgmsg_to_cv2(self.headcam_rgb, "bgr8")
		except CvBridgeError, e:
			print(e)
		else:
			with cd(face_detect_dir):
				cv2.imwrite('camera_img_2.jpg', cv2_img)


	def fromPixelTo3D(self,uv):
		# print(self.headcam_pc.fields)
		# print(self.headcam_pc.data)
		i = uv[1]*self.headcam_pc.row_step + uv[0]*self.headcam_pc.point_step
		print(i)
		for field in self.headcam_pc.fields:
			if field.name == 'x':
				ix = i + field.offset
			if field.name == 'y':
				iy = i + field.offset
			if field.name == 'z':
				iz = i + field.offset
		
		bytes_x = [ord(x) for x in self.headcam_pc.data[ix:ix+4]]
		bytes_y = [ord(x) for x in self.headcam_pc.data[iy:iy+4]]
		bytes_z = [ord(x) for x in self.headcam_pc.data[iz:iz+4]]
		temp = struct.pack('4B',*bytes_x)
		px = struct.unpack('f', temp)[0]
		temp = struct.pack('4B',*bytes_y)
		py = struct.unpack('f', temp)[0]
		temp = struct.pack('4B',*bytes_z)
		pz = struct.unpack('f', temp)[0]

		p = [px,py,pz]
		
		return p

	def get_position(self, req):
		print('Finding location of '+ str(req.person_name)+ '...')
		print('hello?')		
		# print('finding ' + req)
		bounding_box = self.get_face()

		# To do do get face center from bounding box 
		face_center = (240,380)

		p = self.fromPixelTo3D(face_center)

		print(p)

		time =  self.tf_listener.getLatestCommonTime('/map','/head_camera_depth_optical_frame')
		(trans,rot) = self.tf_listener.lookupTransform('/map','/head_camera_depth_optical_frame',time)

		Ttrans = tf.transformations.translation_matrix(trans)
		Trot = tf.transformations.quaternion_matrix(rot)
		Tp = tf.transformations.translation_matrix(p)

		Tp_map = tf.transformations.concatenate_matrices(Ttrans,Trot,Tp)
		p_map = Tp_map[0:3,3]

		print('transform')

		print(p_map)

		position = GetPersonPositionResponse()
		position.x = p_map[0]
		position.y = p_map[1]

		return position



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
