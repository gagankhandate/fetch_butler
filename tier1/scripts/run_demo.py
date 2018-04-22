#!/usr/bin/env python
import rospy
from fetch_gazebo_demo import *
from person_locator.srv import *
import numpy as np

import tf 

def get_person_position(name):
	req = GetPersonPositionRequest(name)
	rospy.wait_for_service('person_locator')
	try:
		get_position = rospy.ServiceProxy('person_locator', GetPersonPosition)
		return get_position(req)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

tf_listener = tf.TransformListener()

if __name__ == '__main__':
	rospy.init_node('goto_person', anonymous=True)
	
	move_base = MoveBaseClient()

	rospy.sleep(5)

	while True:
		
		time =  tf_listener.getLatestCommonTime('/map','/base_link')
		(trans,rot) = tf_listener.lookupTransform('/map','/base_link',time)
		print(trans)
		print(rot)
		euler = tf.transformations.euler_from_quaternion(rot)
		print(euler)

		curr_pos = np.array([trans[0],trans[1],0])

		goal_pos = curr_pos + np.random.random(3)*0.3 + np.array([0.1,0.1,0])

		print("moving to new goal")
		print(goal_pos)
		move_base.goto(goal_pos[0],goal_pos[1],goal_pos[2])












	





