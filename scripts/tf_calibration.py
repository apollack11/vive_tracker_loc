#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from tf import transformations as t
import numpy as np
from operator import mul
import math
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

if __name__ == "__main__":
    rospy.init_node('base_to_lighthouse_broadcaster')
    rate = rospy.Rate(100.0)
    rospy.sleep(5.0)
    br = tf2_ros.StaticTransformBroadcaster()
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

    # transform from gripper to tracker
    transRT_mat = t.translation_matrix(np.array([0.042, 0, -0.02]))
    rotRT_mat = t.quaternion_matrix(t.quaternion_from_euler(math.pi/2, 0, math.pi/2, 'sxyz'))
    matRT = np.dot(transRT_mat, rotRT_mat)

    matBT = np.dot(matBR, matRT)
    matBL = np.dot(matBT, matTL)
    transBL = t.translation_from_matrix(matBL)
    rotBL = t.quaternion_from_matrix(matBL)

    static_transformStamped = TransformStamped()
  
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base"
    static_transformStamped.child_frame_id = "lighthouse"
  
    static_transformStamped.transform.translation = Vector3(*transBL)
    static_transformStamped.transform.rotation = Quaternion(*rotBL)

    br.sendTransform(static_transformStamped)
    rospy.spin()

    # while not rospy.is_shutdown():
    #     br.sendTransform(transBL, rotBL, rospy.Time.now(), "lighthouse", "base")
    #     rate.sleep()
 
