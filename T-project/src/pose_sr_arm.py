#!/usr/bin/env python

import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from moveit_commander import MoveGroupCommander, RobotCommander, \
    PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64MultiArray, String
from rospy import get_rostime
from tf import TransformerROS

flag = False


def set_wall( width, x_position,name):
    """
    Sets a plane for the ground.
    @param height - specifies the height of the plane
    @param z_position - position in z to place the plane. Should not collide with the robot.
    """
    ps = PlanningSceneInterface()
    pose = PoseStamped()
    pose.pose.position.x = x_position 
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    pose.header.stamp = get_rostime()
    pose.header.frame_id = "ra_base_link"
    ps.add_box(name, pose, (width, 3, 3))
    rospy.sleep(2)
    return


def callback(data):
    global flag
    if flag == False:
        flag = True
        time_plan = 1
        time_exec = 1
        hand_commander = SrHandCommander(name="right_hand")
        #################Saved Joint States for Hand and arm##########################
        open_hand = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
             'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
             'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
             'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
             'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 'rh_THJ5': 0.0,
             'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}
	grasp_bottle_water = {'rh_FFJ1': 0.5448908050161929, 'rh_FFJ2': 0.7598270604031128, 'rh_FFJ3': 1.251211647202449, 'rh_FFJ4': 0.001118578356273287, 'rh_THJ4': 1.2217304764, 'rh_THJ5': -0.3921991818293535, 'rh_THJ1': 1.57079632679, 'rh_THJ2': 0.4332412575987424, 'rh_THJ3': 0.15235212377549892, 'rh_LFJ2': 1.0627369514784386, 'rh_LFJ3': 0.9851975210544216, 'rh_LFJ1': 0.7236356317687757, 'rh_LFJ4': -0.08290611360434727, 'rh_LFJ5': 0.04285427800314876, 'rh_RFJ4': 0.00553748487849453, 'rh_RFJ1': 0.6639714736616585, 'rh_RFJ2': 0.8249230709710211, 'rh_RFJ3': 1.2237106475367723, 'rh_MFJ1': 0.248432295769554, 'rh_MFJ3': 1.3106702962070573, 'rh_MFJ2': 0.969627362219072, 'rh_MFJ4': 0.03748994703598271, 'rh_WRJ2': -0.02743155933009626, 'rh_WRJ1': -0.09555055643160751}

        ready = {'rh_FFJ1': 0.08694657087464419, 'rh_FFJ2': 0.8978900878423953, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.12009385114045998, 'rh_THJ4': 1.0123550586629988, 'rh_THJ5': -1.0130694706534844, 'rh_THJ1': 0.0922686710553, 'rh_THJ2': 0.5717864863879395, 'rh_THJ3': 0.209439510239, 'rh_LFJ2': 0.45466343384053465, 'rh_LFJ3': 0.1182807397184345, 'rh_LFJ1': 0.031029331585002762, 'rh_LFJ4': 0.005957012372957621, 'rh_LFJ5': 0.017408288987341338, 'rh_RFJ4': 0.0022346914135096957, 'rh_RFJ1': 0.03361418769975494, 'rh_RFJ2': 0.6149815807734736, 'rh_RFJ3': 0.003988025831215724, 'rh_MFJ1': 0.01222599880755676, 'rh_MFJ3': 0.0, 'rh_MFJ2': 0.7788166145421624, 'rh_MFJ4': 0.07760546976642524, 'rh_WRJ2': -0.030645055281285347, 'rh_WRJ1': -0.14478216537188726}


        grasp =  {'rh_FFJ1': 1.0328779651814242, 'rh_FFJ2': 1.0584467436410925, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.14850204865527247, 'rh_THJ4': 1.0178174299063887, 'rh_THJ5': 0.6386029372568253, 'rh_THJ1': 0.14637546779355942, 'rh_THJ2': -0.03849316144370143, 'rh_THJ3': 0.209439510239, 'rh_LFJ2': 1.2493220785415293, 'rh_LFJ3': 0.006389732482710218, 'rh_LFJ1': 0.38303420792634624, 'rh_LFJ4': 0.007025083214904845, 'rh_LFJ5': 0.007985760016767774, 'rh_RFJ4': 0.014476792419139302, 'rh_RFJ1': 0.09070495093584663, 'rh_RFJ2': 1.3554452174762415, 'rh_RFJ3': 0.011657601887225727, 'rh_MFJ1': 0.12861750745549744, 'rh_MFJ3': 0.004719948297921087, 'rh_MFJ2': 1.4054996330384992, 'rh_MFJ4': 0.08830091051256014, 'rh_WRJ2': -0.019352852890793134, 'rh_WRJ1': -0.16013892711329988}
        home = {'ra_shoulder_pan_joint': -0.9548323790179651, 'ra_elbow_joint': 1.5953612327575684, 'ra_wrist_1_joint': -0.6262839476214808, 'ra_shoulder_lift_joint': -1.0913003126727503, 'ra_wrist_3_joint': 4.8720574378967285, 'ra_wrist_2_joint': 0.556668758392334}

        arm_place = {'ra_shoulder_pan_joint': -0.9872286955462855, 'ra_elbow_joint': 1.8631319999694824, 'ra_wrist_1_joint': -1.0581448713885706, 'ra_shoulder_lift_joint': -0.7090705076800745, 'ra_wrist_3_joint': 4.669511318206787, 'ra_wrist_2_joint': 0.7330016493797302}

	lego_1 = {'rh_FFJ1': 0.73263096005745, 'rh_FFJ2': 1.1402590523283267, 'rh_FFJ3': 0.6150853734180111, 'rh_FFJ4': 0.08437858427989624, 'rh_THJ4': 1.1130691758194835, 'rh_THJ5': -0.5612233193010185, 'rh_THJ1': 0.24306148307103165, 'rh_THJ2': 0.5385986929835019, 'rh_THJ3': 0.13716295333971176, 'rh_LFJ2': 1.0666876464653472, 'rh_LFJ3': 0.6824737181635439, 'rh_LFJ1': 0.4586098430636508, 'rh_LFJ4': -0.009096928231011991, 'rh_LFJ5': 0.007965409946094291, 'rh_RFJ4': 0.00017678316105619444, 'rh_RFJ1': 0.1883461628256109, 'rh_RFJ2': 1.4289179488908414, 'rh_RFJ3': 0.6176262897371383, 'rh_MFJ1': 0.3124965295211516, 'rh_MFJ3': 0.6202848187113245, 'rh_MFJ2': 1.4188762440939302, 'rh_MFJ4': 0.05386570328443786, 'rh_WRJ2': -0.027496051749348063, 'rh_WRJ1': -0.08618491427588047}

	juice_1 =  {'rh_FFJ1': 0.4276920691768282, 'rh_FFJ2': 1.2375706026808344, 'rh_FFJ3': 0.12775783740550314, 'rh_FFJ4': 0.01502254956945965, 'rh_THJ4': 0.8718463355677765, 'rh_THJ5': 0.623152683967913, 'rh_THJ1': 0.8082152192546745, 'rh_THJ2': 0.07649675206998843, 'rh_THJ3': 0.08850656699681927, 'rh_LFJ2': 1.2445095897952214, 'rh_LFJ3': 0.006516947352292049, 'rh_LFJ1': 0.41297931596485543, 'rh_LFJ4': -0.0016858754434753954, 'rh_LFJ5': 0.013789575508597577, 'rh_RFJ4': 0.0023898456482942558, 'rh_RFJ1': 0.5567542901715464, 'rh_RFJ2': 1.3883812694896829, 'rh_RFJ3': 0.01058129975251108, 'rh_MFJ1': 0.5792544649285218, 'rh_MFJ3': 0.008975880262546847, 'rh_MFJ2': 1.3711026331816767, 'rh_MFJ4': 0.08161830629685196, 'rh_WRJ2': -0.027723299538602853, 'rh_WRJ1': -0.06062666113846258}
        arm_commander = SrArmCommander()
        arm_commander.set_planner_id("right_arm")
        arm_commander.set_ground(0.1, -0.34)
        set_wall(0.1, 1.40,"front_wall")
        set_wall(0.1, -0.30,"back_wall")
        arm_commander.set_max_velocity_scaling_factor(0.3)
        arm_commander.set_max_acceleration_scaling_factor(0.3)
######################################################################
        joint_states = ready
        rospy.loginfo("Moving hand to open position")
        hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, False)
        rospy.sleep(1)

##########################Go to Home pose#############################################
        current_pose = arm_commander.get_current_pose()
        goal_pose = current_pose
        goal_pose.position.x = 0.6439
        goal_pose.position.y = -0.484
        goal_pose.position.z = 0.349
        arm_commander.plan_to_pose_target(goal_pose, arm_commander.get_end_effector_link(), False )
        rospy.sleep(time_plan)
        arm_commander.move_to_pose_target(goal_pose, arm_commander.get_end_effector_link())
        rospy.sleep(time_plan)


###########################Go to pick pose############################################
        current_pose = arm_commander.get_current_pose()
        goal_pose = current_pose
        goal_pose.position.x = data.data[0] - 0.38  #data.data[0] - 0.36
        goal_pose.position.y = data.data[1] - 0.03	    #data.data[1] - 0.14
        goal_pose.position.z = data.data[2] + 0.04   #data.data[2] + 0.14
        print "goal pose find:"
        print goal_pose
        arm_commander.plan_to_pose_target(goal_pose, arm_commander.get_end_effector_link(), False )

        print "planning succesful"
        rospy.sleep(time_plan+2)
        arm_commander.move_to_pose_target(goal_pose, arm_commander.get_end_effector_link())
        #arm_commander.clear_pose_targets()
        rospy.sleep(time_plan)
#######################################################################

        joint_states = grasp
        rospy.loginfo("Moving hand to grasp position")
        hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, False)
        rospy.sleep(time_plan)
##########################Go to Home pose#############################################
        current_pose = arm_commander.get_current_pose()
        goal_pose = current_pose
        goal_pose.position.x = 0.6439
        goal_pose.position.y = -0.484
        goal_pose.position.z = 0.349

        arm_commander.plan_to_pose_target(goal_pose, arm_commander.get_end_effector_link(), False )
        rospy.sleep(time_plan)
        arm_commander.move_to_pose_target(goal_pose, arm_commander.get_end_effector_link())
        #arm_commander.clear_pose_targets()
        rospy.sleep(time_plan)


#########################Go to place pose##############################################
        current_pose = arm_commander.get_current_pose()
        goal_pose = current_pose
        goal_pose.position.x = 0.600
        goal_pose.position.y = -0.484
        goal_pose.position.z = -.0350

        arm_commander.plan_to_pose_target(goal_pose, arm_commander.get_end_effector_link(), False )
        rospy.sleep(time_plan+2)
        arm_commander.move_to_pose_target(goal_pose, arm_commander.get_end_effector_link())
        #arm_commander.clear_pose_targets()
        rospy.sleep(time_plan)
#######################################################################


        joint_states = ready
        rospy.loginfo("Moving hand to open position")
        hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, False)
        rospy.sleep(1)


##########################Go to Home pose#############################################
        current_pose = arm_commander.get_current_pose()
        goal_pose = current_pose
        goal_pose.position.x = 0.6439
        goal_pose.position.y = -0.484
        goal_pose.position.z = 0.349

        arm_commander.plan_to_pose_target(goal_pose, arm_commander.get_end_effector_link(), False )
        rospy.sleep(time_plan)
        arm_commander.move_to_pose_target(goal_pose, arm_commander.get_end_effector_link())
        #arm_commander.clear_pose_targets()
        rospy.sleep(10)
#######################################################################

        rospy.signal_shutdown("Task Completed!")


if __name__=='__main__':
  try:
    rospy.init_node("basic_hand_arm_example", anonymous=True)
    rospy.Subscriber("/tf_transform/tf_result", Float64MultiArray , callback)
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
