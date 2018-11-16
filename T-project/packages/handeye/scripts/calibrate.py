#!/usr/bin/env python

import rospy
from handeye import HandEyeConnectorTF


def main():
	rospy.init_node('hand_eye')

	hec = HandEyeConnectorTF()

	rospy.spin()
	

if __name__ == '__main__':
 	main() 