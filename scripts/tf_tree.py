#!/usr/bin/env python
# Copyright 2022 Lucas Walter
#
# print the current tf tree to logs

import sys

import rospy
# import tf2_ros
# import tf2_py as tf2
from tf2_msgs.msg import TFMessage


class TfTree(object):
    def __init__(self):
        frames_pre = []
        for arg in sys.argv[1:]:
            if arg[0] == '_':
                continue
            frames_pre.append(arg)
        frames_pre = list(set(frames_pre))
        print(frames_pre)

        # how long to listen to tf topics
        wait = rospy.get_param("~wait", 1.0)
        max_indent = rospy.get_param("~max_indent", 18)
        self.parents = {}
        self.children = {}
        self.publishers = {}
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_callback, queue_size=100)
        self.tf_static_sub = rospy.Subscriber("/tf_static", TFMessage, self.tf_callback, queue_size=100)
        rospy.sleep(rospy.Duration(wait))
        self.tf_sub.unregister()

        frames = []
        for frame in frames_pre:
            if frame in self.children.keys():
                frames.append(frame)
            # rospy.logwarn(f"'{frame}' not in tf {self.children.keys()}")

        roots = {}
        if len(frames) != 1:
            for parent, children in self.parents.items():
                if len(children) > 0 and parent not in self.children.keys():
                    roots[parent] = children
        else:
            # only print the tree below the one frame
            roots[frames[0]] = self.parents[frames[0]]

        # show only the tf tree connecting all the provided frames
        # TODO(lucasw) this also shows the common parents of all frames
        # all the way to the root, but could eliminate those
        lineages = {}
        common_lineage = None
        parent_mask = []
        if len(frames) > 1:
            for frame in frames:
                lineages[frame] = self.get_lineage(frame)
                if common_lineage is None:
                    common_lineage = lineages[frame]
                else:
                    common_lineage = [x for x in common_lineage if x in lineages[frame]]
                parent_mask.extend(lineages[frame])
            print(f"common lineage: {common_lineage}")
            if len(common_lineage) > 0:
                roots = {}
                roots[common_lineage[0]] = self.parents[common_lineage[0]]
            parent_mask = list(set(parent_mask))
        rospy.loginfo(f"parent mask {parent_mask}")

        rospy.loginfo(list(roots.keys()))
        for root in roots.keys():
            self.print(root, max_indent=max_indent, parent_mask=parent_mask)

    # get every parent of frame in a list
    def get_lineage(self, frame, lineage=None):
        if lineage is None:
            lineage = [frame]
        if frame in self.children.keys():
            parent = self.children[frame]
            lineage.append(parent)
            return self.get_lineage(parent, lineage)
        return lineage

    def print(self, parent, indent=0, max_indent=12, parent_mask=[]):
        if len(parent_mask) > 0 and parent not in parent_mask:
            return
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
                self.print(child, indent + 1, max_indent=max_indent, parent_mask=parent_mask)

    def tf_callback(self, msg):
        # TODO(lucasw) look in roswtf and the tf view_frame tree generator for a more robust
        # approach, note update rates and multiple parents for one frame issues
        for tfs in msg.transforms:
            parent = tfs.header.frame_id
            child = tfs.child_frame_id

            if parent not in self.parents.keys():
                self.parents[parent] = []
            if child not in self.parents.keys():
                self.parents[child] = []

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
