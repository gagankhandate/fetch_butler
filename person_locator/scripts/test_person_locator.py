#!/usr/bin/env python

import rospy
from person_locator.srv import *

def get_person_position(name):
	req = GetPersonPositionRequest(name)
	print(req)
	rospy.wait_for_service('person_locator')
	try:
		get_position = rospy.ServiceProxy('person_locator', GetPersonPosition)
		return get_position(req)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == '__main__':
	rospy.init_node('test_person_locator', anonymous=True)
	
	rate = rospy.Rate(0.2)
	while True:	
		result = get_person_position('Thanos')
		print(result)
		rate.sleep()