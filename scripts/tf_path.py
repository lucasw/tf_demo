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
from nav_msgs.msg import Path


class TfPath(object):
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)

        self.parent = rospy.get_param("~parent", "map")
        self.child = rospy.get_param("~child", "base_link")
        self.cache_time = rospy.get_param("~cache_time", 30.0)

        self.pub = rospy.Publisher("path", Path, queue_size=2)
        self.path = Path()
        self.path.header.frame_id = self.parent

        update_rate = rospy.get_param("~update_rate", 5.0)
        rospy.loginfo("tf path poses expire after {}s".format(self.cache_time))
        self.timer = rospy.Timer(rospy.Duration(1.0 / update_rate), self.update)

    def update(self, event):
        lookup_time = rospy.Time(0)
        try:
            tf = self.tf_buffer.lookup_transform(self.parent, self.child, lookup_time)
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as ex:
            # TODO(lucasw) only warn on edge
            rospy.logwarn_throttle(5.0, lookup_time.to_sec())
            rospy.logdebug_throttle(5.0, traceback.format_exc())
            rospy.logwarn_throttle(5.0, ex)
            return

        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        self.path.poses.append(pose)

        test_time = event.current_real
        if (test_time - self.path.poses[0].header.stamp).to_sec() > self.cache_time:
            self.path.poses = self.path.poses[1:]

        self.path.header.stamp = tf.header.stamp
        self.pub.publish(self.path)


if __name__ == '__main__':
    rospy.init_node('tf_path')
    node = TfPath()
    rospy.spin()
