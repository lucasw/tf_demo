#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

buffer_core = tf2_ros.BufferCore(rospy.Duration(10.0))
ts1 = TransformStamped()
ts1.header.stamp = rospy.Time(413)
ts1.header.frame_id = 'map'
ts1.child_frame_id = 'frame1'
ts1.transform.translation.x = 2.71828183
ts1.transform.rotation.w = 1.0
# TODO(lucasw) does the authority matter at all?  Could it be set to anything?
buffer_core.set_transform(ts1, "default_authority")

# print(dir(buffer_core))
# why no lookup_transform(
a = buffer_core.lookup_transform_core('map', 'frame1', rospy.Time(0))
print(a)
# ((2.71828183, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
b = buffer_core.lookup_transform_core('frame1', 'map', rospy.Time(0))
print(b)
# ((-2.71828183, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

print("=================")

ts2 = TransformStamped()
ts2.header.stamp = rospy.Time(0)
ts2.header.frame_id = 'frame1'
ts2.child_frame_id = 'frame2'
ts2.transform.translation.x = 0
ts2.transform.translation.y = 0.5
# TODO(lucasw) example rotation using transform3d/transformations.py
ts2.transform.rotation.w = 1.0
buffer_core.set_transform(ts2, "default_authority")

print(buffer_core.lookup_transform_core('map', 'frame2', rospy.Time(0)))
