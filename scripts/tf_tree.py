#!/usr/bin/env python
# Copyright 2022 Lucas Walter
#
# print the current tf tree to logs

import rospy
# import tf2_ros
# import tf2_py as tf2
from tf2_msgs.msg import TFMessage


class TfTree(object):
    def __init__(self):
        wait = rospy.get_param("~wait", 1.0)
        max_indent = rospy.get_param("~max_indent", 18)
        self.parents = {}
        self.children = {}
        self.publishers = {}
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_callback, queue_size=100)
        self.tf_static_sub = rospy.Subscriber("/tf_static", TFMessage, self.tf_callback, queue_size=100)
        rospy.sleep(rospy.Duration(wait))
        self.tf_sub.unregister()

        roots = {}
        for parent, children in self.parents.items():
            if parent not in self.children.keys():
                roots[parent] = children

        rospy.loginfo(roots.keys())
        for root in roots.keys():
            self.print(root, max_indent=max_indent)

    def print(self, parent, indent=0, max_indent=12):
        if indent > max_indent:
            print('...')
            return
        text = ''
        for i in range(indent - 1):
            text += '  '
        if indent > 0:
            text += '--'
        text += parent
        if parent in self.publishers.keys():
            text += f" {list(self.publishers[parent].keys())}"
        print(text)
        if parent in self.parents.keys():
            for child in self.parents[parent]:
                self.print(child, indent + 1, max_indent=max_indent)

    def tf_callback(self, msg):
        # TODO(lucasw) look in roswtf and the tf view_frame tree generator for a more robust
        # approach, note update rates and multiple parents for one frame issues
        for tfs in msg.transforms:
            parent = tfs.header.frame_id
            child = tfs.child_frame_id

            if parent not in self.parents.keys():
                self.parents[parent] = []
            if child not in self.parents[parent]:
                self.parents[parent].append(child)

            self.children[child] = parent
            rospy.logdebug_once(msg._connection_header)
            publisher = msg._connection_header['callerid']
            if child not in self.publishers.keys():
                self.publishers[child] = {}
            self.publishers[child][publisher] = True


if __name__ == '__main__':
    rospy.init_node('tf_tree')
    node = TfTree()
