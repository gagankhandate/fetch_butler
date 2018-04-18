#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2

from my_service.srv import *

bridge = CvBridge()
class ImageHandler(object):
	def __init__(self):
		self.need_image = True

	def rgb_listener(self):
		rospy.init_node('GetCameraPics', anonymous=True)
		rospy.Subscriber("/head_camera/rgb/image_raw", Image, self.callback)
		print('listening to head camera rgb')
		rospy.spin()

	def callback(self, data):
		print('image callback was called')
		#rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
		try:
			cv2_img = bridge.imgmsg_to_cv2(data, "rgb8")
		except CvBridgeError, e:
			print(e)
		else:
			cv2.imwrite('camera_img.jpeg', cv2_img)

	def run(self):
		print('camera pics started')
		self.rgb_listener()
		#while True:
		

if __name__ == '__main__':
	new_handler = ImageHandler()
	new_handler.run()
