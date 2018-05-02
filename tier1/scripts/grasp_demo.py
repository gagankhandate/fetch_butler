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

if __name__ == '__main__':
	rospy.init_node("simple_disco")

	object_pose = get_object('Pringles')

	# Create move group interface for a fetch robot
	move_group = MoveGroupInterface("arm_with_torso", "base_link")

	move_group_gripper = MoveGroupInterface("","")

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
	table_height = 0.6
	planning_scene.addCube("table",table_height,object_pose.g_pos.position.x + 0.2,object_pose.g_pos.position.y,-0.5*table_height)

	print('created secene')

	Lwrist2gripper = 0.15
	gripper_frame = 'wrist_roll_link'

	# Creating msg
	gripper_pose_grasp = PoseStamped()
	gripper_pose_grasp.header.frame_id = "base_link"
	gripper_pose_grasp.pose = object_pose.g_pos
	gripper_pose_grasp.pose.position.x = gripper_pose_grasp.pose.position.x -  Lwrist2gripper
	gripper_pose_pregrasp = gripper_pose_grasp
	gripper_pose_pregrasp.pose.position.x = gripper_pose_pregrasp.pose.position.x -  0.1

	print('created msgs')

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


	move_group_gripper.set_named_target('close')
	close_plan = move_group_gripper.plan()
	move_group_gripper.execute(close_plan)

	print('gripper closed?')
	
	move_group.get_move_action().cancel_all_goals()







