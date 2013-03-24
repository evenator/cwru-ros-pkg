#! /usr/bin/python

import roslib; roslib.load_manifest('cwru_utilities')
import rospy
import tf
import tf.transformations as transformations

rospy.init_node('printGripperTip')
tf_listener = tf.TransformListener()
tf_listener.waitForTransform("/irb_120_base_link","/gripper_jaw_2_tip", rospy.Time(0),rospy.Duration(10,0))
(trans, rot) = tf_listener.lookupTransform("/irb_120_base_link","/gripper_jaw_2_tip", rospy.Time(0))
print "{{ {},{},{},{},{},{},{} }},".format(trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3])
