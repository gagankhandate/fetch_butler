#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

from my_service.srv import *

class GetNumsSrv(object):
 
	def __init__(self):
		self.num1 = 0
		self.num2 = 0

	def store_num1(self, data):
		self.num1 = data.data
    	# rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

	def store_num2(self, data):
		self.num2 = data.data
		
	def	listener(self):
		rospy.Subscriber("nums/num1", Float64, self.store_num1)
		rospy.Subscriber("nums/num1", Float64, self.store_num2)
		# rospy.spin()
	
	def handle(self, req):
		result = GetNumsResponse()
		result.num1 = self.num1
		result.num2 = self.num2
		return result
    
get_nums_srv = GetNumsSrv()
	
if __name__ == '__main__':
	rospy.init_node('get_nums_service', anonymous=True)
	s = rospy.Service('get_nums', GetNums, get_nums_srv.handle)
	print('service started ...')
	rate = rospy.Rate(2)
	while True:
		get_nums_srv.listener()	
		rate.sleep()
	
