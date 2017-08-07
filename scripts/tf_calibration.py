#!/usr/bin/env python
import rospy
import tf
from tf import transformations as t
import numpy as np
from operator import mul
import math

if __name__ == "__main__":
    rospy.init_node('base_to_lighthouse_broadcaster')
    rate = rospy.Rate(10.0)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    listener = tf.TransformListener()
    listener.waitForTransform("right_gripper", "base", rospy.Time(0), rospy.Duration(20.0))
    (transBR, rotBR) = listener.lookupTransform("base", "right_gripper", rospy.Time(0))
    listener.waitForTransform("lighthouse", "tracker", rospy.Time(0), rospy.Duration(20.0))
    (transTL, rotTL) = listener.lookupTransform("tracker", "lighthouse", rospy.Time(0))

    transBR_mat = t.translation_matrix(transBR)
    rotBR_mat = t.quaternion_matrix(rotBR)
    matBR = np.dot(transBR_mat, rotBR_mat)    

    transTL_mat = t.translation_matrix(transTL)
    rotTL_mat = t.quaternion_matrix(rotTL)
    matTL = np.dot(transTL_mat, rotTL_mat)

    matBL = np.dot(matBR, matTL)
    transBL = t.translation_from_matrix(matBL)
    rotBL = t.quaternion_from_matrix(matBL)

    while not rospy.is_shutdown():
        # br.sendTransform((2.0, 0, 2.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "lighthouse", "base")
        br.sendTransform(transBL, rotBL, rospy.Time.now(), "lighthouse", "base")
        rate.sleep()
