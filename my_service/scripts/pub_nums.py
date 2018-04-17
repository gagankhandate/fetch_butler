#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

pub1 = rospy.Publisher('nums/num1', Float64, queue_size=10)
pub2 = rospy.Publisher('nums/num2', Float64, queue_size=10)
rospy.init_node('publish_numbers')
r = rospy.Rate(10) # 10hz
i = 0
while not rospy.is_shutdown():
	i+= 1
	pub1.publish(i)
	pub2.publish(i)
	r.sleep()

