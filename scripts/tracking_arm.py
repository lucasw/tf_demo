#!/usr/bin/env python
# Lucas Walter
# February 2018

import math
import numpy
import rospy
import tf2_ros
import tf2_py as tf2

from geometry_msgs.msg import Quaternion, TransformStamped
from tf import transformations


def quat_from_quat_array(quat_array):
    quat = Quaternion()
    quat.x = quat_array[0]
    quat.y = quat_array[1]
    quat.z = quat_array[2]
    quat.w = quat_array[3]
    return quat


def quat_from_euler(roll, pitch, yaw):
    quat_array = transformations.quaternion_from_euler(roll, pitch, yaw)
    return quat_from_quat_array(quat_array)


rospy.init_node('tracking_arm')

tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20))
tl = tf2_ros.TransformListener(tf_buffer)

br = tf2_ros.TransformBroadcaster()

parent = rospy.get_param("~parent", "map")
target = rospy.get_param("~target", "flyer")
delay = rospy.get_param("~delay", 0.5)

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
ts['finger'].transform.translation.z = 0.0

base_angle = 0.0
while not rospy.is_shutdown():
    cur_time = rospy.Time.now()
    for key in ts.keys():
        ts[key].header.stamp = cur_time
        br.sendTransform(ts[key])

    rospy.sleep(0.03)

    if True:
        base_angle += 0.0005
        ts['base'].transform.rotation = quat_from_euler(0, 0, base_angle)
        elbow_angle = (math.pi + math.cos(cur_time.to_sec() * 0.05)) * 0.3
        ts['elbow'].transform.rotation = quat_from_euler(0, elbow_angle, 0)

        wrist_pitch = (math.cos(cur_time.to_sec() * 0.07)) * 0.4
        wrist_yaw = (math.cos(cur_time.to_sec() * 0.11)) * 0.24
        ts['wrist'].transform.rotation = quat_from_euler(0, wrist_pitch, wrist_yaw)

    #
    # finger_pitch = (math.cos(cur_time.to_sec() * 2.7)) * 0.4
    # ts['finger'].transform.rotation = quat_from_euler(0, finger_pitch, 0)

    # get the relative transform between the wrist and target
    lookup_time = cur_time - rospy.Duration(delay)
    # if tf_buffer.can_transform('wrist', target, lookup_time):
    if True:
        try:
            # target_trans = tf_buffer.lookup_transform(target, 'wrist', lookup_time)
            target_trans = tf_buffer.lookup_transform('wrist', target, lookup_time)
            # rospy.loginfo(target_trans)
        except tf2.LookupException as ex:
            rospy.logerr(ex)
            continue
        except tf2.ExtrapolationException as ex:
            rospy.logerr(ex)
            continue
        # else:
        #     rospy.logerr("can't transform")
        # ts['finger'].transform.rotation = target_trans.transform.rotation
        # Need to construct a rotation out of the xyz translation
        rot_mat = numpy.identity(4)
        vector = [target_trans.transform.translation.x,
                  target_trans.transform.translation.y,
                  target_trans.transform.translation.z]

        # vector[2] = 0.0
        forward = vector / numpy.linalg.norm(vector)
        up = [0, 0, 1]
        left = numpy.cross(up, forward)
        left = left / numpy.linalg.norm(left)
        up = numpy.cross(forward, left)
        rot_mat[0][0:3] = forward
        rot_mat[1][0:3] = left
        rot_mat[2][0:3] = up
        rot_mat = rot_mat.transpose()
        quat_array = transformations.quaternion_from_matrix(rot_mat)
        ts['finger'].transform.rotation = quat_from_quat_array(quat_array)
        # rospy.loginfo(target_trans)
    else:
        rospy.logwarn("can't transform")
