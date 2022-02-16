#!/usr/bin/env python
# Lucas Walter
# February 2018
#
# Look up a transform between parent and child, then republish
# with a new child

import copy

import rospy
import tf2_ros
import tf2_py as tf2
# import traceback
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from geometry_msgs.msg import TransformStamped


class OldTfToNewTf(object):
    def __init__(self):
        cache_time = rospy.get_param("~cache_time", 30.0)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(cache_time))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)

        self.br = tf2_ros.TransformBroadcaster()

        update_rate = rospy.get_param("~update_rate", 20.0)

        self.last_lookup_failed = None
        self.last_lookup_time = None

        self.config = None
        self.ddr = DDynamicReconfigure("")
        self.ddr.add_variable("lookup_time_offset", "offset the lookup time", 0.0, -10.0, 10.0)
        self.ddr.add_variable("lookup_time_most_recent", "use the most recent tf", True)
        self.ddr.add_variable("lookup_parent", "lookup parent", "map")
        self.ddr.add_variable("lookup_child", "lookup child", "child")
        self.ddr.add_variable("broadcast_time_offset", "offset the broadcast time", 0.0, -10.0, 10.0)
        self.ddr.add_variable("broadcast_parent", "broadcast parent", "map")
        self.ddr.add_variable("broadcast_child", "broadcast child", "child2")
        # TODO(lucasw) zero roll, pitch, yaw?
        self.ddr.add_variable("zero_rotation", "zero out rotation", False)
        self.ddr.add_variable("zero_x", "zero out x", False)
        self.ddr.add_variable("zero_y", "zero out y", False)
        self.ddr.add_variable("zero_z", "zero out z", False)
        self.ddr.start(self.config_callback)

        # collect some transforms:
        rospy.sleep(1.0)

        self.timer = rospy.Timer(rospy.Duration(1.0 / update_rate), self.update)

    def config_callback(self, config, level):
        self.config = config
        return config

    def update(self, event):
        config = copy.deepcopy(self.config)
        if config is None:
            return

        ts = TransformStamped()
        ts.transform.rotation.w = 1.0
        ts.header.frame_id = config.broadcast_parent
        ts.child_frame_id = config.broadcast_child

        cur_time = event.current_real
        if config.lookup_time_most_recent:
            lookup_time = rospy.Time(0)
        else:
            lookup_time = cur_time + rospy.Duration(config)

        try:
            trans = self.tf_buffer.lookup_transform(config.lookup_parent, config.lookup_child, lookup_time)
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as ex:
            if self.last_lookup_failed is not True:
                rospy.logwarn_throttle(2.0, f"lookup time: {lookup_time.to_sec():0.2f}")
                rospy.logwarn_throttle(2.0, ex)
                # rospy.logwarn(traceback.format_exc())
                self.last_lookup_failed = True
            return
        if self.last_lookup_failed is not False:
            rospy.logwarn_throttle(2.0, f"now looking up {trans.header.frame_id} to {trans.child_frame_id}")
        self.last_lookup_failed = False

        if trans.header.stamp == self.last_lookup_time:
            return
        self.last_lookup_time = trans.header.stamp

        # rospy.loginfo_throttle(5.0, trans.transform.translation.x)

        ts.transform = trans.transform
        ts.header.stamp = trans.header.stamp + rospy.Duration(config.broadcast_time_offset)

        if config.zero_rotation:
            ts.transform.rotation.x = 0.0
            ts.transform.rotation.y = 0.0
            ts.transform.rotation.z = 0.0
            ts.transform.rotation.w = 1.0
        if config.zero_x:
            ts.transform.translation.x = 0.0
        if config.zero_y:
            ts.transform.translation.y = 0.0
        if config.zero_z:
            ts.transform.translation.z = 0.0

        rospy.logdebug_once(ts)

        self.br.sendTransform(ts)


if __name__ == '__main__':
    rospy.init_node('old_tf_to_new_tf')
    node = OldTfToNewTf()
    rospy.spin()
