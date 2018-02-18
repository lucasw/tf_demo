#!/usr/bin/env python
# Lucas Walter
# February 2018

import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped


rospy.init_node("flip_flop")

br = tf2_ros.TransformBroadcaster()
ts = TransformStamped()
ts.header.frame_id = "map"
ts.child_frame_id = "frame1"
ts.transform.rotation.w = 1.0
ts.transform.translation.x = 1.0
ts.transform.translation.y = 1.0

while not rospy.is_shutdown():
    ts.transform.translation.x *= -1.0
    ts.header.stamp = rospy.Time.now()
    br.sendTransform(ts)
    rospy.sleep(0.5)
