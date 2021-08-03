#!/usr/bin/env python
# Lucas Walter
# February 2018
#
# Look up a transform between parent and child, then republish
# with a new child

import rospy
import tf2_ros
import tf2_py as tf2
import traceback

from geometry_msgs.msg import TransformStamped


rospy.init_node('old_tf_to_new_tf')

cache_time = rospy.get_param("~cache_time", 30.0)
tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(cache_time))
tl = tf2_ros.TransformListener(tf_buffer)

br = tf2_ros.TransformBroadcaster()
ts = TransformStamped()
ts.transform.rotation.w = 1.0

offset = rospy.get_param("~offset1", -1.0)
parent = rospy.get_param("~parent1", "map")
child = rospy.get_param("~child1", "frame1")

offset2 = rospy.get_param("~offset2", 0.0)
ts.header.frame_id = rospy.get_param("parent2", parent)
ts.child_frame_id = rospy.get_param("~child2", child + "_b")

while not rospy.is_shutdown():
    rospy.sleep(0.01)
    cur_time = rospy.Time.now()
    lookup_time = cur_time + rospy.Duration(offset)
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
    # rospy.loginfo_throttle(5.0, trans.transform.translation.x)

    ts.transform = trans.transform
    ts.header.stamp = cur_time + rospy.Duration(offset2)
    br.sendTransform(ts)
