#!/usr/bin/env python

import rospy
from object_locator.srv import *

def get_object(name):
	req = GetObjectPoseRequest(name)
	print(req)
	rospy.wait_for_service('object_locator')
	try:
		get_position = rospy.ServiceProxy('object_locator', GetObjectPose)
		return get_position(req)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == '__main__':
	rospy.init_node('test_person_locator', anonymous=True)
	
	rate = rospy.Rate(1)
	while True:
		name = 'pringles'
		result = get_object(name)
		print(result)
		rate.sleep()