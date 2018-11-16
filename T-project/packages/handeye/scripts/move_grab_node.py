#!/usr/bin/env python

import rospy, actionlib
import tf
import tf2_ros
import tf_conversions.posemath as tfconv
import PyKDL
import yaml
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Vector3, Quaternion, Transform, TransformStamped
from compute_ik import calculate_inverse_kinematics, choose_proper_ik, go_to_q_pos
from control_msgs.msg import FollowJointTrajectoryAction

def main():
	rospy.init_node('move_grab_node')
	client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
	print('Waiting for server...')
	client.wait_for_server()
	print("Connected to server")

	while not rospy.is_shutdown():
		raw_input("Hit [ENTER] to go back to home.")
		home_pos = np.array([-0.4, 0, 0.20])
  		goal_rot = np.array([[0, 1, 0],
  							[1, 0, 0],
  							[0, 0, -1]])

		num_sols, q_sols = calculate_inverse_kinematics(home_pos, goal_rot)
		sol, flag = choose_proper_ik(num_sols, q_sols)
		go_to_q_pos(client, list(sol), 5)

		goal = rospy.wait_for_message('/object/pose', Pose)
		raw_input("Message received, hit [ENTER] if you are ready to move...")
		goal_pos = np.array([goal.position.x, goal.position.y, goal.position.z])
		goal_rot_mat = tf.transformations.quaternion_matrix([goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w])
		num_sols, q_sols = calculate_inverse_kinematics(home_pos, goal_rot_mat[:3, :3])
		sol, flag = choose_proper_ik(num_sols, q_sols)
		go_to_q_pos(client, list(sol), 5)
		



if __name__ == '__main__':
	main()
