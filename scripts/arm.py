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

rospy.init_node('tf_lookup')

# tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20))
# tl = tf2_ros.TransformListener(tf_buffer)

br = tf2_ros.TransformBroadcaster()

parent = rospy.get_param("~parent", "map")

ts = {}
ts['base'] = TransformStamped()
ts['base'].transform.rotation.w = 1.0
ts['base'].header.frame_id = parent
ts['base'].child_frame_id = 'base'
ts['base'].transform.translation.x = 1.0

ts['elbow'] = TransformStamped()
ts['elbow'].transform.rotation.w = 1.0
ts['elbow'].header.frame_id = 'base'
ts['elbow'].child_frame_id = 'elbow'
ts['elbow'].transform.translation.z = 1.0

ts['wrist'] = TransformStamped()
ts['wrist'].transform.rotation.w = 1.0
ts['wrist'].header.frame_id = 'elbow'
ts['wrist'].child_frame_id = 'wrist'
ts['wrist'].transform.translation.z = 0.7

ts['finger'] = TransformStamped()
ts['finger'].transform.rotation.w = 1.0
ts['finger'].header.frame_id = 'wrist'
ts['finger'].child_frame_id = 'finger'
ts['finger'].transform.translation.z = 0.1

base_angle = 0.0
while not rospy.is_shutdown():
    rospy.sleep(0.01)
    cur_time = rospy.Time.now()

    base_angle += 0.0005
    ts['base'].transform.rotation = quat_from_euler(0, 0, base_angle)
    elbow_angle = (math.pi + math.cos(cur_time.to_sec() * 0.5)) * 0.3
    ts['elbow'].transform.rotation = quat_from_euler(0, elbow_angle, 0)

    wrist_pitch = (math.cos(cur_time.to_sec() * 0.7)) * 0.4
    wrist_yaw = (math.cos(cur_time.to_sec() * 1.1)) * 0.24
    ts['wrist'].transform.rotation = quat_from_euler(0, wrist_pitch, wrist_yaw)

    finger_pitch = (math.cos(cur_time.to_sec() * 2.7)) * 0.4
    ts['finger'].transform.rotation = quat_from_euler(0, finger_pitch, 0)

    for key in ts.keys():
        ts[key].header.stamp = cur_time
        br.sendTransform(ts[key])
