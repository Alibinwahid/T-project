import rospy
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from ur_control.srv import *

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def go_to_q_pos(client, goal_angle, time):

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    joint_states = rospy.wait_for_message("joint_states", JointState)
    joints_pos = joint_states.position

    g.trajectory.points = []
    g.trajectory.points.append(
        JointTrajectoryPoint(positions=joints_pos, velocities=[0] * 6, time_from_start=rospy.Duration(0.0)))
    g.trajectory.points.append(
        JointTrajectoryPoint(positions=goal_angle, velocities=[0] * 6, time_from_start=rospy.Duration(time)))

    client.send_goal(g)
    client.wait_for_result()
    return 0







