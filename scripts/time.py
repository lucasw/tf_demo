#!/usr/bin/env python
# get the current ros time (so uses sim time) and print using date formatting

import datetime
import rospy

rospy.init_node("time")

date_format = rospy.get_param("~format", "+%Y%m%d_%H%M%S")
use_format = rospy.get_param("~use_format", True)

ts_epoch = rospy.Time.now().to_sec()

if use_format:
    ts = datetime.datetime.fromtimestamp(ts_epoch).strftime(date_format)
else:
    ts = ts_epoch

print(ts)
