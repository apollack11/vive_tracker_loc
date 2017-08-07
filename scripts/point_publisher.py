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
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped


if __name__ == "__main__":
    rospy.init_node('point_republisher')
    rate = rospy.Rate(50.0)
    listener = tf.TransformListener()
    rospy.sleep(5.0)
    gripper_pub = rospy.Publisher("gripper_pose", PoseStamped, queue_size=1)
    tracker_pub = rospy.Publisher("tracker_pose", PoseStamped, queue_size=1)

    # rospy.loginfo("Waiting for gripper to base")
    # for i in range(10):
    #     listener.waitForTransform("right_gripper", "base", rospy.Time(0), rospy.Duration(2.0))
    #     rospy.loginfo("still waiting on gripper->base")
    #     rospy.sleep(2.0)
    # if i > 8:
    #     rospy.logwarn("never found gripper to base")
    #     rospy.signal_shutdown("Sawyer tf error")

    # rospy.loginfo("Waiting for tracker to lighthouse")
    # for i in range(10):
    #     listener.waitForTransform("tracker", "lighthouse", rospy.Time(0), rospy.Duration(4.0))
    #     rospy.loginfo("still waiting on tracker->lighthouse")
    # if i > 8:
    #     rospy.logwarn("never found tracker to lighthouse")
    #     rospy.signal_shutdown("Vive tf error")
        
        
    while not rospy.is_shutdown():
        try:
            (transBR, rotBR) = listener.lookupTransform("base", "right_gripper", rospy.Time(0))
            (transLT, rotLT) = listener.lookupTransform("lighthouse", "tracker", rospy.Time(0))
            p = PoseStamped()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = "base"
            p.pose.position = Point(*transBR)
            p.pose.orientation = Quaternion(*rotBR)
            gripper_pub.publish(p)
            p.header.frame_id = "lighthouse"
            p.pose.position = Point(*transLT)
            p.pose.orientation = Quaternion(*rotLT)
            tracker_pub.publish(p)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("tf lookup error")
        rate.sleep()

    # while not rospy.is_shutdown():
    #     br.sendTransform(transBL, rotBL, rospy.Time.now(), "lighthouse", "base")
    #     rate.sleep()
 
