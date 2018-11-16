#!/usr/bin/env python

import rospy
import tf

if __name__ == '__main__':
	rospy.init_node('test')
	listener = tf.TransformListener()
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
	    time = rospy.Time.now()
	    listener.waitForTransform("base_link", "ee_link", rospy.Time(0),  rospy.Duration(0.1))
	    (trans, rot) = listener.lookupTransform("base_link", "ee_link", rospy.Time(0))
	    print(trans)
	    rate.sleep()

				

