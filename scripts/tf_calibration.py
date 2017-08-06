#!/usr/bin/env python
import rospy
import tf
from tf import transformations as t
import numpy as np
from operator import mul

if __name__ == "__main__":
    rospy.init_node('base_to_lighthouse_broadcaster')
    rate = rospy.Rate(10.0)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    listener = tf.TransformListener()
    listener.waitForTransform("base", "right_gripper", rospy.Time(0), rospy.Duration(10.0))
    (transBR, rotBR) = listener.lookupTransform("base", "right_gripper", rospy.Time(0))
    listener.waitForTransform("lighthouse", "tracker", rospy.Time(0), rospy.Duration(10.0))
    (transTL, rotTL) = listener.lookupTransform("tracker", "lighthouse", rospy.Time(0))
    (transLT, rotLT) = listener.lookupTransform("lighthouse", "tracker", rospy.Time(0))

    transTL = np.multiply(transLT, -1)
    transBL = tuple(map(sum, zip(transBR, transTL)))

    # transBR_mat = t.translation_matrix(transBR)
    rotBR_mat = t.quaternion_matrix(rotBR)
    rotTL_mat = t.quaternion_matrix(rotTL)
    rotBL_mat = np.dot(rotBR_mat, rotTL_mat)
    rotBL = t.quaternion_from_matrix(rotBL_mat)

    # matBR = np.dot(transBR_mat, rotBR_mat)



    # transTL_mat = t.translation_matrix(transTL)
    # rotTL_mat = t.quaternion_matrix(rotTL)
    # matTL = np.dot(transTL_mat, rotTL_mat)

    # matBL = np.dot(matBR, matTL)
    # transBL = t.translation_from_matrix(matBL)
    # rotBL = t.quaternion_from_matrix(matBL)

    print transBR
    print transTL
    print transBL
    print rotBL

    while not rospy.is_shutdown():
        br.sendTransform(transBL, (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "lighthouse", "base")
        rate.sleep()
