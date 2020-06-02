#!/usr/bin/env python
# Lucas Walter
# February 2018

import math
import rospy
import tf2_ros
import tf2_py as tf2

from geometry_msgs.msg import Quaternion, TransformStamped
from tf import transformations


def quat_from_euler(roll, pitch, yaw):
    quat_array = transformations.quaternion_from_euler(roll, pitch, yaw)
    quat = Quaternion()
    quat.x = quat_array[0]
    quat.y = quat_array[1]
    quat.z = quat_array[2]
    quat.w = quat_array[3]
    return quat


rospy.init_node('flyer')

# tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20))
# tl = tf2_ros.TransformListener(tf_buffer)

br = tf2_ros.TransformBroadcaster()

parent = rospy.get_param("~parent", "map")

ts = TransformStamped()
ts.transform.rotation.w = 1.0
ts.header.frame_id = parent
ts.child_frame_id = 'flyer'

vel = 0.1

while not rospy.is_shutdown():
    rospy.sleep(0.01)
    cur_time = rospy.Time.now()
    t = cur_time.to_sec() * 0.5
    # t = 0

    ts.transform.translation.x = 5.0 + 4.0 * math.cos(t * 0.1) + 1.5 * math.cos(t * 0.471)
    ts.transform.translation.y = 1.0 + 6.0 * math.sin(t * 0.1)
    ts.transform.translation.z = 7.0 + math.cos(t * 0.031)
    # ts.transform.rotation = quat_from_euler(0, 0, base_angle)

    ts.header.stamp = cur_time
    br.sendTransform(ts)
