#!/usr/bin/env python
# Lucas Walter
# February 2018

import rospy
import tf2_ros
import tf2_py as tf2

from geometry_msgs.msg import TransformStamped


rospy.init_node('tf_lookup')

tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20))
tl = tf2_ros.TransformListener(tf_buffer)

br = tf2_ros.TransformBroadcaster()
ts = TransformStamped()
ts.header.frame_id = "map"
ts.transform.rotation.w = 1.0
ts.transform.translation.x = 1.0
ts.transform.translation.y = 1.0

offset = rospy.get_param("~offset", -1.0)
parent = rospy.get_param("~parent", "map")
child = rospy.get_param("~child", "frame1")
ts.child_frame_id = rospy.get_param("~child_old", "frame1_old")

while not rospy.is_shutdown():
    rospy.sleep(0.01)
    lookup_time = rospy.Time.now() + rospy.Duration(offset)
    try:
        trans = tf_buffer.lookup_transform(parent, child, lookup_time)
    except tf2.LookupException as ex:
        print lookup_time.to_sec()
        print ex
        continue
    except tf2.ExtrapolationException as ex:
        print lookup_time.to_sec()
        print ex
        continue
    # print trans.transform.translation.x

    ts.transform = trans.transform
    ts.header.stamp = lookup_time
    br.sendTransform(ts)
