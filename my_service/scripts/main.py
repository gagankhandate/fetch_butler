#!/usr/bin/env python

import rospy
from my_service.srv import GetNums

if __name__ == '__main__':
	rospy.init_node('get_nums_service', anonymous=True)
	get_nums_s = rospy.ServiceProxy('get_nums', GetNums)
	rate = rospy.Rate(5)
	while True:	
		result = get_nums_s('adas')
		print(result)
		rate.sleep()
	
	
	
