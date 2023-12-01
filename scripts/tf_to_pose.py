#!/usr/bin/env python
"""
Copyright 2023 Lucas Walter

Look up a transform between parent and child, then republish as a PoseStamped,
this is convenient for viewing orientation as rpy in degrees in PlotJuggler

rosrun tf_demo flyer.py
rosrun tf_demo tf_to_pose.py _parent:=map _child:=flyer

"""

import copy
from threading import Lock

import rospy
import tf2_ros
import tf2_py as tf2
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from geometry_msgs.msg import (
    PoseStamped,
    TransformStamped,
)


class TfToPose:
    def __init__(self):
        self.lock = Lock()

        cache_time = rospy.get_param("~cache_time", 30.0)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(cache_time))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)

        update_rate = rospy.get_param("~update_rate", 20.0)

        self.last_lookup_failed = None
        self.last_lookup_time = None

        self.config = None
        self.ddr = DDynamicReconfigure("")
        self.ddr.add_variable("lookup_time_offset", "offset the lookup time", 0.0, -10.0, 10.0)
        self.ddr.add_variable("lookup_time_offset_parent",
                              "offset the parent lookup time, need a reference frame", 0.0, -10.0, 10.0)
        self.ddr.add_variable("lookup_time_most_recent", "use the most recent tf", True)
        self.ddr.add_variable("parent", "lookup parent", "map")
        self.ddr.add_variable("child", "lookup child", "child")
        self.ddr.add_variable("reference_frame", "reference frame if lookup_time_offset_parent != 0.0", "map")

        self.ddr.start(self.config_callback)

        self.pose_pub = rospy.Publisher("tf_pose", PoseStamped, queue_size=3)

        # collect some transforms:
        try:
            rospy.sleep(1.0)
        except rospy.exceptions.ROSTimeMovedBackwardsException as ex:
            rospy.logwarn(ex)

        self.timer = rospy.Timer(rospy.Duration(1.0 / update_rate), self.update, reset=True)

    def config_callback(self, config, level):
        with self.lock:
            self.config = config
        return config

    def update(self, event):
        with self.lock:
            config = copy.deepcopy(self.config)
        if config is None:
            return

        cur_time = event.current_real
        ts = self.get_transform(config, cur_time)

        if ts is None:
            return

        if ts.header.stamp == self.last_lookup_time:
            rospy.logwarn_once(f"{ts.header.stamp.to_sec():0.3f}s {cur_time.to_sec():0.3f}s")
            return None
        self.last_lookup_time = ts.header.stamp

        rospy.logdebug_once(ts)

        ps = PoseStamped(header=ts.header)
        ps.pose.position.x = ts.transform.translation.x
        ps.pose.position.y = ts.transform.translation.y
        ps.pose.position.z = ts.transform.translation.z
        ps.pose.orientation = ts.transform.rotation

        self.pose_pub.publish(ps)

    # TODO(lucasw) put this into a library, old_tf_to_new_tf also uses it
    def get_default_transform(self) -> TransformStamped:
        ts = TransformStamped()
        ts.transform.rotation.w = 1.0
        return ts

    def get_transform(self, config, cur_time) -> TransformStamped:
        ts = self.get_default_transform()
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

        if config.parent == config.child and lookup_time_parent == lookup_time:
            rospy.loginfo_once("detected same parent and child at same stamp, returning identity")
            # TODO(lucasw) the lookup returns the same timestamp as the last lookup
            # only returns rospy.Time(0) the first pass, when doing this lookup
            # with same parent and child- so avoid that and return an identity transform
            ts = self.get_default_transform()
            ts.header.stamp = cur_time
            return ts

        try:
            text = f"'{config.parent}' {lookup_time_parent.to_sec():0.2f}s"
            text += f" to '{config.child}' {lookup_time.to_sec():0.2f}s"
            text += f", reference '{config.reference_frame}'"
            rospy.loginfo_once(text)
            if config.lookup_time_offset_parent != 0.0:
                trans = self.tf_buffer.lookup_transform_full(config.parent, lookup_time_parent,
                                                             config.child, lookup_time,
                                                             config.reference_frame,
                                                             rospy.Duration(0.15))
            else:
                trans = self.tf_buffer.lookup_transform(config.parent, config.child, lookup_time)
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as ex:
            if self.last_lookup_failed is not True:
                rospy.logwarn_throttle(2.0, f"lookup time: {lookup_time.to_sec():0.2f}")
                rospy.logwarn_throttle(2.0, ex)
                self.last_lookup_failed = True
            return None
        except rospy.exceptions.ROSTimeMovedBackwardsException as ex:
            rospy.logwarn(ex)
            return None
        if self.last_lookup_failed is not False:
            rospy.logwarn_throttle(2.0, f"now looking up {trans.header.frame_id} to {trans.child_frame_id}")
        self.last_lookup_failed = False

        ts.transform = trans.transform
        # this can't be a static transform if it is being updated all the time
        if trans.header.stamp == rospy.Time(0):
            rospy.logwarn_once("setting timestamp to current time (probably a static lookup)")
            trans.header.stamp = cur_time  # TODO(lucasw) optionally add an offset?
        ts.header = trans.header

        return ts


def main():
    rospy.init_node("tf_to_pose")
    _ = TfToPose()
    rospy.spin()


if __name__ == "__main__":
    main()
