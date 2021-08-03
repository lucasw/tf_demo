#!/usr/bin/env python
# Lucas Walter
# February 2018
#
# Look up a transform between parent and child periodically, publish as Path
# similar to hector_trajectory_server but with max buffer length

import rospy
import tf2_ros
import tf2_py as tf2
import traceback

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path

rospy.init_node('old_tf_to_new_tf')

tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
tl = tf2_ros.TransformListener(tf_buffer)

parent = rospy.get_param("~parent", "map")
child = rospy.get_param("~child", "base_link")
update_rate = rospy.get_param("~update_rate", 5.0)
cache_time = rospy.get_param("~cache_time", 30.0)

pub = rospy.Publisher("path", Path, queue_size=2)
path = Path()
path.header.frame_id = parent

while not rospy.is_shutdown():
    rospy.sleep(1.0 / update_rate)
    lookup_time = rospy.Time(0)
    try:
        trans = tf_buffer.lookup_transform(parent, child, lookup_time)
    except tf2.LookupException as ex:
        # TODO(lucasw) only warn on edge
        rospy.logwarn_throttle(5.0, lookup_time.to_sec())
        rospy.logwarn_throttle(5.0, traceback.format_exc())
        continue
    except tf2.ExtrapolationException as ex:
        rospy.logwarn_throttle(5.0, lookup_time.to_sec())
        rospy.logwarn_throttle(5.0, traceback.format_exc())
        continue

    pose = PoseStamped()
    pose.header = trans.header
    pose.pose.position.x = trans.transform.translation.x
    pose.pose.position.y = trans.transform.translation.y
    pose.pose.position.z = trans.transform.translation.z
    pose.pose.orientation = trans.transform.rotation
    path.poses.append(pose)

    if (trans.header.stamp - path.poses[0].header.stamp).to_sec() > cache_time:
        path.poses = path.poses[1:]

    path.header.stamp = trans.header.stamp
    pub.publish(path)
