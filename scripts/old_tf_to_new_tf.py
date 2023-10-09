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
from tf2_msgs.msg import TFMessage


class OldTfToNewTf(object):
    def __init__(self):
        cache_time = rospy.get_param("~cache_time", 30.0)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(cache_time))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)

        # self.br = tf2_ros.TransformBroadcaster()
        # can't use remapping because that will remap the transform listener above
        tf_pub_topic = rospy.get_param("~tf_pub_topic", "/tf")
        self.tf_pub = rospy.Publisher(tf_pub_topic, TFMessage, queue_size=10)

        update_rate = rospy.get_param("~update_rate", 20.0)

        self.last_lookup_failed = None
        self.last_lookup_time = None

        self.config = None
        self.ddr = DDynamicReconfigure("")
        self.ddr.add_variable("lookup_time_offset", "offset the lookup time", 0.0, -10.0, 10.0)
        self.ddr.add_variable("lookup_time_offset_parent",
                              "offset the parent lookup time, need a reference frame", 0.0, -10.0, 10.0)
        self.ddr.add_variable("lookup_time_most_recent", "use the most recent tf", True)
        self.ddr.add_variable("publish_if_lookup_fails", "publish zero tf if lookup fails", False)
        self.ddr.add_variable("lookup_parent", "lookup parent", "map")
        self.ddr.add_variable("lookup_child", "lookup child", "child")
        self.ddr.add_variable("reference_frame", "reference frame if lookup_time_offset_parent != 0.0", "map")
        self.ddr.add_variable("broadcast_time_offset", "offset the broadcast time", 0.0, -10.0, 10.0)
        self.ddr.add_variable("broadcast_parent", "broadcast parent", "map")
        self.ddr.add_variable("broadcast_child", "broadcast child", "child2")
        # TODO(lucasw) zero roll, pitch, yaw?
        self.ddr.add_variable("zero_rotation", "zero out rotation", False)
        self.ddr.add_variable("zero_x", "zero out x", False)
        self.ddr.add_variable("zero_y", "zero out y", False)
        self.ddr.add_variable("zero_z", "zero out z", False)

        self.ddr.add_variable("offset_x", "offset output x", 0.0, -10.0, 10.0)
        self.ddr.add_variable("offset_y", "offset output y", 0.0, -10.0, 10.0)
        self.ddr.add_variable("offset_z", "offset output z", 0.0, -10.0, 10.0)

        self.ddr.start(self.config_callback)

        # collect some transforms:
        try:
            rospy.sleep(1.0)
        except rospy.exceptions.ROSTimeMovedBackwardsException as ex:
            rospy.logwarn(ex)

        self.timer = rospy.Timer(rospy.Duration(1.0 / update_rate), self.update, reset=True)

    def config_callback(self, config, level):
        self.config = config
        return config

    def update(self, event):
        config = copy.deepcopy(self.config)
        if config is None:
            return

        cur_time = event.current_real
        ts = self.get_transform(config, cur_time)

        if ts is None:
            if not config.publish_if_lookup_fails:
                return
            text = "this will publish a possibly reasonable looking transform when tf is failing for good reasons"
            rospy.logwarn_once(text)
            ts = self.get_default_transform(config)
            ts.header.stamp = cur_time

        if ts.header.stamp == self.last_lookup_time:
            rospy.logwarn_once(f"{ts.header.stamp.to_sec():0.3f}s {cur_time.to_sec():0.3f}s")
            if not config.publish_if_lookup_fails:
                return None
            # TODO(lucasw) may should use the last transform instead
            ts = self.get_default_transform(config)
            ts.header.stamp = cur_time
        self.last_lookup_time = ts.header.stamp

        ts.header.stamp += rospy.Duration(config.broadcast_time_offset)

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

        ts.transform.translation.x += config.offset_x
        ts.transform.translation.y += config.offset_y
        ts.transform.translation.z += config.offset_z

        rospy.logdebug_once(ts)

        tfm = TFMessage()
        tfm.transforms.append(ts)
        self.tf_pub.publish(tfm)

        # self.br.sendTransform(ts)

    def get_default_transform(self, config) -> TransformStamped:
        ts = TransformStamped()
        ts.transform.rotation.w = 1.0
        ts.header.frame_id = config.broadcast_parent
        ts.child_frame_id = config.broadcast_child
        return ts

    def get_transform(self, config, cur_time) -> TransformStamped:
        ts = self.get_default_transform(config)
        if config.lookup_time_most_recent:
            lookup_time = rospy.Time(0)
        else:
            lookup_time = cur_time + rospy.Duration(config.lookup_time_offset)

        if config.lookup_time_offset_parent != 0.0:
            if lookup_time == rospy.Time(0):
                rospy.logerr("need to use lookup_time_most_recent:=false")
            # TODO(lucasw) if lookup_time is rospy.Time(0) then need to get the most recent
            # available time, then add the duration
            lookup_time_parent = lookup_time + rospy.Duration(config.lookup_time_offset_parent)
        else:
            lookup_time_parent = lookup_time

        if config.lookup_parent == config.lookup_child and lookup_time_parent == lookup_time:
            rospy.loginfo_once("detected same parent and child at same stamp, returning identity")
            # TODO(lucasw) the lookup returns the same timestamp as the last lookup
            # only returns rospy.Time(0) the first pass, when doing this lookup
            # with same parent and child- so avoid that and return an identity transform
            ts = self.get_default_transform(config)
            ts.header.stamp = cur_time
            return ts

        try:
            text = f"'{config.lookup_parent}' {lookup_time_parent.to_sec():0.2f}s"
            text += f" to '{config.lookup_child}' {lookup_time.to_sec():0.2f}s"
            text += f", reference '{config.reference_frame}'"
            rospy.loginfo_once(text)
            if config.lookup_time_offset_parent != 0.0:
                trans = self.tf_buffer.lookup_transform_full(config.lookup_parent, lookup_time_parent,
                                                             config.lookup_child, lookup_time,
                                                             config.reference_frame,
                                                             rospy.Duration(0.15))
            else:
                trans = self.tf_buffer.lookup_transform(config.lookup_parent, config.lookup_child, lookup_time)
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as ex:
            if self.last_lookup_failed is not True:
                rospy.logwarn_throttle(2.0, f"lookup time: {lookup_time.to_sec():0.2f}")
                rospy.logwarn_throttle(2.0, ex)
                # rospy.logwarn(traceback.format_exc())
                self.last_lookup_failed = True
            return None
        except rospy.exceptions.ROSTimeMovedBackwardsException as ex:
            rospy.logwarn(ex)
            return None
        if self.last_lookup_failed is not False:
            rospy.logwarn_throttle(2.0, f"now looking up {trans.header.frame_id} to {trans.child_frame_id}")
        self.last_lookup_failed = False

        # rospy.loginfo_throttle(5.0, trans.transform.translation.x)

        ts.transform = trans.transform
        # this can't be a static transform if it is being updated all the time
        if trans.header.stamp == rospy.Time(0):
            rospy.logwarn_once("setting timestamp to current time (probably a static lookup)")
            trans.header.stamp = cur_time  # TODO(lucasw) optionally add an offset?
        ts.header.stamp = trans.header.stamp

        return ts


if __name__ == '__main__':
    rospy.init_node('old_tf_to_new_tf')
    node = OldTfToNewTf()
    rospy.spin()
