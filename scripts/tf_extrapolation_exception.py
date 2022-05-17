#!/usr/bin/env python

import rospy
import tf2_ros
import traceback
from geometry_msgs.msg import TransformStamped


bc = tf2_ros.BufferCore()

tfs_a = TransformStamped()
tfs_a.header.stamp += rospy.Duration(10.0)
tfs_a.header.frame_id = "map"
tfs_a.child_frame_id = "base"
tfs_a.transform.rotation.w = 1.0

bc.set_transform(tfs_a, "tf_test")
tfs_a.header.stamp += rospy.Duration(2.0)
bc.set_transform(tfs_a, "tf_test")

for offset in [-3.0, -1.0, 2.0]:
    try:
        tfs_b = bc.lookup_transform_core("map", "base", tfs_a.header.stamp + rospy.Duration(offset))
    except Exception as ex:
        print(ex)
        print(traceback.print_exc())
    print("-----------------------")
