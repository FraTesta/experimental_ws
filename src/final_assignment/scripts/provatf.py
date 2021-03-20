#!/usr/bin/env python
import tf 
import rospy 
import time

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

   

    rate = rospy.Rate(10.0)
    time.sleep(5)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

	rospy.loginfo("il  valore di x è")
	rospy.loginfo(trans[0])
        #rospy.loginfo(trans)

        rate.sleep()
