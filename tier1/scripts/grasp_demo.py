#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from object_locator.srv import *
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from moveit_commander import MoveGroupCommander

# move_group_gripper = MoveGroupCommander('gripper')


def get_object(name):
	req = GetObjectPoseRequest(name)
	print(req)
	rospy.wait_for_service('object_locator')
	try:
		get_position = rospy.ServiceProxy('object_locator', GetObjectPose)
		return get_position(req)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def get_pose_from_msg(pose_msg):

	x = [pose_msg.g_pos.position.x,pose_msg.g_pos.position.y,pose_msg.g_pos.position.z]

	q = [pose_msg.g_pos.orientation.x,pose_msg.g_pos.orientation.y,pose_msg.g_pos.orientation.z,pose_msg.g_pos.orientation.w] 

	return x,q

def create_pose_stamped_msg(x,q,frame_id):

	pose_msg = PoseStamped()
	pose_msg.pose.position.x = x[0]
	pose_msg.pose.position.y = x[1]
	pose_msg.pose.position.z = x[2]

	pose_msg.pose.orientation.x = q[0]
	pose_msg.pose.orientation.y = q[1]
	pose_msg.pose.orientation.z = q[2]
	pose_msg.pose.orientation.w = q[3]

	pose_msg.header.frame_id = frame_id

	return pose_msg

if __name__ == '__main__':
	rospy.init_node("simple_disco")

	object_pose = get_object('Pringles')
	print(object_pose)

	X,Q = get_pose_from_msg(object_pose)

	print(X)
	print(Q)

	# Define ground plane
	# This creates objects in the planning scene that mimic the ground
	# If these were not in place gripper could hit the ground
	planning_scene = PlanningSceneInterface("base_link")
	planning_scene.removeCollisionObject("my_front_ground")
	planning_scene.removeCollisionObject("my_back_ground")
	planning_scene.removeCollisionObject("my_right_ground")
	planning_scene.removeCollisionObject("my_left_ground")
	planning_scene.removeCollisionObject("table")
	planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
	planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
	planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
	planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
	
	table_height = 0.7
	planning_scene.addCube("table",table_height,X[0] + 0.3,X[1],0.5*table_height)

	print('created secene')

	# Create move group interface for a fetch robot
	move_group = MoveGroupInterface("arm_with_torso", "base_link")
	move_group_gripper = MoveGroupInterface("gripper_custom","gripper_link")
	gripper_frame = 'wrist_roll_link'


	Lwrist2gripper = 0.12
	X_pg = [X[0]- Lwrist2gripper - 0.20,X[1],X[2]]
	X_g = [X[0] - Lwrist2gripper,X[1],X[2]]
	X_up = [X_g[0],X_g[1],X_g[2]+0.2]

	print(X_pg)
	print(X_g)


	# Open gripper
	move_group_gripper.moveToJointPosition(['l_gripper_finger_joint','l_gripper_finger_joint'],[0.040, 0.040],wait=False)
	move_group_gripper.get_move_action().wait_for_result()
	result = move_group_gripper.get_move_action().get_result()

	
	gripper_pose_pregrasp = create_pose_stamped_msg(X_pg,Q,'base_link')

	gripper_pose_grasp = create_pose_stamped_msg(X_g,Q,'base_link')

	gripper_pose_up = create_pose_stamped_msg(X_up,Q,'base_link')

	# # Create pre grasp pose msg
	# gripper_pose_pregrasp = PoseStamped()
	# gripper_pose_pregrasp.header.frame_id = "base_link"
	# # do position and orientation seperately
	# gripper_pose_pregrasp.pose.position = object_pose.g_pos.position
	# gripper_pose_pregrasp.pose.orientation = object_pose.g_pos.orientation
	# gripper_pose_pregrasp.pose.position.x = g_pos_x - Lwrist2gripper - 0.20 

	# print(object_pose)

	# # Creating grasp pose msg
	# gripper_pose_grasp = PoseStamped()
	# gripper_pose_grasp.header.frame_id = "base_link"
	# gripper_pose_grasp.pose.position = object_pose.g_pos.position
	# gripper_pose_grasp.pose.orientation = object_pose.g_pos.orientation
	# gripper_pose_grasp.pose.position.x = g_pos_x -  Lwrist2gripper

	# print(object_pose)
	
	print('created msgs')
	print(gripper_pose_pregrasp)
	print(gripper_pose_grasp)
	print(gripper_pose_up)

	# Pregrasp
	gripper_pose_pregrasp.header.stamp = rospy.Time.now()
	move_group.moveToPose(gripper_pose_pregrasp, gripper_frame)
	result = move_group.get_move_action().get_result()

	print('moving to pregrasp')

	if result:
		# Checking the MoveItErrorCode
		if result.error_code.val == MoveItErrorCodes.SUCCESS:
			rospy.loginfo("Moved to Pregrasp")
			print()
		else:
			# If you get to this point please search for:
			# moveit_msgs/MoveItErrorCodes.msg
			rospy.logerr("Arm goal in state: %s",
						 move_group.get_move_action().get_state())
	else:
		rospy.logerr("MoveIt! failure no result returned.")

	print('moving to grasp')
	gripper_pose_grasp.header.stamp = rospy.Time.now()
	move_group.moveToPose(gripper_pose_grasp, gripper_frame)
	result = move_group.get_move_action().get_result()
	if result:
		# Checking the MoveItErrorCode
		if result.error_code.val == MoveItErrorCodes.SUCCESS:
			rospy.loginfo("Moved to Grasp Pose")
		else:
			# If you get to this point please search for:
			# moveit_msgs/MoveItErrorCodes.msg
			rospy.logerr("Arm goal in state: %s",
						 move_group.get_move_action().get_state())
	else:
		rospy.logerr("MoveIt! failure no result returned.")

	print('closing gripper')


	move_group_gripper.moveToJointPosition(['l_gripper_finger_joint','l_gripper_finger_joint'],[0.013, 0.013],wait=False)
	move_group_gripper.get_move_action().wait_for_result()
	result = move_group_gripper.get_move_action().get_result()

	if result:
		# Checking the MoveItErrorCode
		if result.error_code.val == MoveItErrorCodes.SUCCESS:
			rospy.loginfo("Grasped!")
		else:
			# If you get to this point please search for:
			# moveit_msgs/MoveItErrorCodes.msg
			rospy.logerr("Arm goal in state: %s",
						 move_group.get_move_action().get_state())
	else:
		rospy.logerr("MoveIt! failure no result returned.")

	# close_plan = move_group_gripper.plan()
	# move_group_gripper.execute(close_plan)

	print('gripper closed?')
	
	gripper_pose_up.header.stamp = rospy.Time.now()
	move_group.moveToPose(gripper_pose_up, gripper_frame)
	result = move_group.get_move_action().get_result()
	if result:
		# Checking the MoveItErrorCode
		if result.error_code.val == MoveItErrorCodes.SUCCESS:
			rospy.loginfo("Moved to Grasp Pose")
		else:
			# If you get to this point please search for:
			# moveit_msgs/MoveItErrorCodes.msg
			rospy.logerr("Arm goal in state: %s",
			move_group.get_move_action().get_state())
	else:
		rospy.logerr("MoveIt! failure no result returned.")


	# move_group.get_move_action().cancel_all_goals()
	
	# move_group_gripper.get_move_action().cancel_all_goals()







