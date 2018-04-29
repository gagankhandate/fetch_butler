#!/usr/bin/env python
import rospy
from fetch_gazebo_demo import *
from person_locator.srv import *
import numpy as np

import tf 

def add_marker_to_scene(goal_pos_x,goal_pos_y):
	import visualization_msgs.msg
	import geometry_msgs.msg
	from geometry_msgs.msg import Point, Quaternion, Pose
	marker = visualization_msgs.msg.Marker()
	marker.type = visualization_msgs.msg.Marker.CUBE
	marker.action = visualization_msgs.msg.Marker.ADD
	marker.lifetime = rospy.Duration(0) # Show block until it is deleted
	marker.scale.x = 0.10
	marker.scale.y = 0.10
	marker.scale.z = 0.10
	marker.color.a = 1
	marker.color.r = 1
	# marker.color = constants.HIGHLIGHTED_BLOCK_COLOR
	marker.id = 1 # "highlighted"

	pose_stamped = geometry_msgs.msg.PoseStamped()
	pose_stamped.header.frame_id = "map"
	pose_stamped.pose = Pose(Point(goal_pos_x,goal_pos_y, 0.0), Quaternion(0,0,0,1))
	marker.pose = pose_stamped.pose
	marker.header = pose_stamped.header

	ma = visualization_msgs.msg.MarkerArray()
	ma.markers.append(marker)

	publisher = rospy.Publisher('marker_goal', visualization_msgs.msg.MarkerArray, queue_size=10)
	publisher.publish(ma)



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
		
	time =  tf_listener.getLatestCommonTime('/map','/base_link')
	(trans,rot) = tf_listener.lookupTransform('/map','/base_link',time)
	print(trans)
	print(rot)
	euler = tf.transformations.euler_from_quaternion(rot)
	print(euler)

	curr_pos = np.array([trans[0],trans[1],0])

	# goal_pos = curr_pos + np.random.random(3)*0.3 + np.array([0.1,0.1,0])

	result = get_person_position('Gagan_Khandate')

	if result is not None:
		d = np.linalg.norm(np.array([result.position.x - curr_pos[0],result.position.y - curr_pos[1]]))
		d0 = 1
		if d < d0:
			print("person too close")
		else:
			goal_pos_x = ((d-d0)/d)*(result.position.x - curr_pos[0]) + curr_pos[0]
			goal_pos_y = ((d-d0)/d)*(result.position.y - curr_pos[1]) + curr_pos[1]
			goal_pos_theta = np.arctan2(goal_pos_y - curr_pos[1],goal_pos_x - curr_pos[0])
			print("moving to new goal")
			print([goal_pos_x,goal_pos_y,goal_pos_theta])
			# move_base.goto(goal_pos[0],goal_pos[1],goal_pos[2])
			add_marker_to_scene(goal_pos_x,goal_pos_y)
			move_base.goto(goal_pos_x,goal_pos_y,goal_pos_theta)








	





