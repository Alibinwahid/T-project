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


rospy.init_node("basic_hand_arm_example", anonymous=True)

#################### Grasp Bottle ##############################################################################################
grasp_bottle_water = {'rh_FFJ1': 0.5448908050161929, 'rh_FFJ2': 0.7598270604031128, 'rh_FFJ3': 1.251211647202449, 'rh_FFJ4': 0.001118578356273287, 'rh_THJ4': 1.2217304764, 'rh_THJ5': -0.3921991818293535, 'rh_THJ1': 1.57079632679, 'rh_THJ2': 0.4332412575987424, 'rh_THJ3': 0.15235212377549892, 'rh_LFJ2': 1.0627369514784386, 'rh_LFJ3': 0.9851975210544216, 'rh_LFJ1': 0.7236356317687757, 'rh_LFJ4': -0.08290611360434727, 'rh_LFJ5': 0.04285427800314876, 'rh_RFJ4': 0.00553748487849453, 'rh_RFJ1': 0.6639714736616585, 'rh_RFJ2': 0.8249230709710211, 'rh_RFJ3': 1.2237106475367723, 'rh_MFJ1': 0.248432295769554, 'rh_MFJ3': 1.3106702962070573, 'rh_MFJ2': 0.969627362219072, 'rh_MFJ4': 0.03748994703598271, 'rh_WRJ2': -0.02743155933009626, 'rh_WRJ1': -0.09555055643160751}

#################### Ready ######################################################################################################
ready = {'rh_FFJ1': 0.08694657087464419, 'rh_FFJ2': 0.8978900878423953, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.12009385114045998, 'rh_THJ4': 1.0123550586629988, 'rh_THJ5': -1.0130694706534844, 'rh_THJ1': 0.0922686710553, 'rh_THJ2': 0.5717864863879395, 'rh_THJ3': 0.209439510239, 'rh_LFJ2': 0.45466343384053465, 'rh_LFJ3': 0.1182807397184345, 'rh_LFJ1': 0.031029331585002762, 'rh_LFJ4': 0.005957012372957621, 'rh_LFJ5': 0.017408288987341338, 'rh_RFJ4': 0.0022346914135096957, 'rh_RFJ1': 0.03361418769975494, 'rh_RFJ2': 0.6149815807734736, 'rh_RFJ3': 0.003988025831215724, 'rh_MFJ1': 0.01222599880755676, 'rh_MFJ3': 0.0, 'rh_MFJ2': 0.7788166145421624, 'rh_MFJ4': 0.07760546976642524, 'rh_WRJ2': -0.030645055281285347, 'rh_WRJ1': -0.14478216537188726}

#################### New Hand Pose ##############################################################################################

new_hand_pose = {'rh_FFJ1': 0.06391436666944045, 'rh_FFJ2': 0.11876752714790684, 'rh_FFJ3': 0.37092126520448504, 'rh_FFJ4': -0.042851687307874656, 'rh_THJ4': 1.1559147919189503, 'rh_THJ5': -0.2588703959296407, 'rh_THJ1': 1.57079632679, 'rh_THJ2': 0.5307312542465946, 'rh_THJ3': -0.019192654039800246, 'rh_LFJ2': 0.7885587193869752, 'rh_LFJ3': 0.0, 'rh_LFJ1': 0.04069420535738072, 'rh_LFJ4': -0.16084349326605735, 'rh_LFJ5': 0.05948008843777471, 'rh_RFJ4': 0.00943046986611886, 'rh_RFJ1': 0.0538894120265912, 'rh_RFJ2': 1.555595072019462, 'rh_RFJ3': 1.2521634364090852, 'rh_MFJ1': 0.0273862373289272, 'rh_MFJ3': 1.3572670844830317, 'rh_MFJ2': 1.5583751879577106, 'rh_MFJ4': 0.048652088844851754, 'rh_WRJ2': -0.029578136086299642, 'rh_WRJ1': -0.15007138163557687}

#################### New Hand Pose ##############################################################################################

lego_1 = {'rh_FFJ1': 0.73263096005745, 'rh_FFJ2': 1.1402590523283267, 'rh_FFJ3': 0.6150853734180111, 'rh_FFJ4': 0.08437858427989624, 'rh_THJ4': 1.1130691758194835, 'rh_THJ5': -0.5612233193010185, 'rh_THJ1': 0.24306148307103165, 'rh_THJ2': 0.5385986929835019, 'rh_THJ3': 0.13716295333971176, 'rh_LFJ2': 1.0666876464653472, 'rh_LFJ3': 0.6824737181635439, 'rh_LFJ1': 0.4586098430636508, 'rh_LFJ4': -0.009096928231011991, 'rh_LFJ5': 0.007965409946094291, 'rh_RFJ4': 0.00017678316105619444, 'rh_RFJ1': 0.1883461628256109, 'rh_RFJ2': 1.4289179488908414, 'rh_RFJ3': 0.6176262897371383, 'rh_MFJ1': 0.3124965295211516, 'rh_MFJ3': 0.6202848187113245, 'rh_MFJ2': 1.4188762440939302, 'rh_MFJ4': 0.05386570328443786, 'rh_WRJ2': -0.027496051749348063, 'rh_WRJ1': -0.08618491427588047}



hand_commander = SrHandCommander(name="right_hand")

joint_states = new_hand_pose
rospy.loginfo("Moving hand to grasp position")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, True)
rospy.sleep(4)

joint_states = ready
rospy.loginfo("Moving hand to grasp position")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, True)
rospy.sleep(10)

joint_states = lego_1
rospy.loginfo("Moving hand to grasp position")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, True)

