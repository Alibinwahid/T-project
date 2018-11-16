#!/usr/bin/env python

import rospy, actionlib
import tf
import tf2_ros
import tf_conversions.posemath as tfconv
import PyKDL
import yaml
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped
from compute_ik import calculate_inverse_kinematics, choose_proper_ik, go_to_q_pos
from control_msgs.msg import FollowJointTrajectoryAction

def main():
	rospy.init_node('grab_cube_node')
	client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
	print('Waiting for server...')
	client.wait_for_server()
	print("Connected to server")

	file_name = rospy.get_param('~data_file_name')
	camera_frame = rospy.get_param('~camera_frame')
	base_frame 	 = rospy.get_param('~base_frame')
	marker_frame = rospy.get_param('~marker_frame')

	# Load the camera-base transform  
	with open(file_name, 'r') as outfile:
		try:
			data = yaml.load(outfile)
		except yaml.YAMLError as exc:
			print(exc)

	# Broadcast static transform between /base_link and /handeye/camera/color_optical_frame 
	broadcaster = tf2_ros.StaticTransformBroadcaster()
	static_transformStamped = TransformStamped()
	static_transformStamped.header.stamp = rospy.Time.now()
	static_transformStamped.header.frame_id = base_frame
	static_transformStamped.child_frame_id = camera_frame

	static_transformStamped.transform.translation.x = data['position']['x']
	static_transformStamped.transform.translation.y = data['position']['y']
	static_transformStamped.transform.translation.z = data['position']['z']

	static_transformStamped.transform.rotation.x = data['orientation']['x']
	static_transformStamped.transform.rotation.y = data['orientation']['y']
	static_transformStamped.transform.rotation.z = data['orientation']['z']
	static_transformStamped.transform.rotation.w = data['orientation']['w']

	broadcaster.sendTransform(static_transformStamped)

	# Wait for the command to move the UR10 to grab the cube
	listener = tf.TransformListener()

	while not rospy.is_shutdown():
		raw_input("Hit [ENTER] to go back to home.")
		home_pos = np.array([-0.6, 0, 0.06])
		# goal_rot = np.array([[0, 0, 1],
  #                            [1, 0, 0],
  #                            [0, 1, 0]])
  		goal_rot = np.array([[0, 1, 0],
  							[1, 0, 0],
  							[0, 0, -1]])

		num_sols, q_sols = calculate_inverse_kinematics(home_pos, goal_rot)
		sol, flag = choose_proper_ik(num_sols, q_sols)
		go_to_q_pos(client, list(sol), 5)

		raw_input("Hit [ENTER] if you have reset cube...")
		listener.waitForTransform(
			base_frame, marker_frame,
			rospy.Time(0), rospy.Duration(0.1))

		(trans, rot) = listener.lookupTransform(
			base_frame, marker_frame, rospy.Time(0))

		print(trans)

		c = raw_input("Hit [ENTER] to grab the cube, hit [c] to continue, hit [q] to quit...")
		if (c == 'q') or (c == 'Q'):
			break
		elif (c == 'c') or (c == 'C'):
			continue

		# Drive the robot to move
		#rot_matrix = tf.transformations.quaternion_matrix(rot)[:3,:3]  # 3 by 3
		num_sols, q_sols = calculate_inverse_kinematics([-trans[0], -trans[1], 0.10], goal_rot)
		sol, flag = choose_proper_ik(num_sols, q_sols)
		go_to_q_pos(client, list(sol), 5)





if __name__ == '__main__':
	main()