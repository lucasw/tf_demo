/*
Copyright 2022 Lucas Walter

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1.
Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <string>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "old_tf_to_new_tf");
  ros::NodeHandle nh;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  std::string frame0 = "odom";
  if (argc > 1) {
    frame0 = argv[1];
  }
  std::string frame1 = "base_link";
  if (argc > 2) {
    frame1 = argv[2];
  }
  ROS_INFO_STREAM(frame0 << " " << frame1);

  while (ros::ok()) {
    ros::Duration(1.0).sleep();
    try {
      const auto tfs = tf_buffer.lookupTransform(frame0, frame1, ros::Time(0));
      ROS_INFO_STREAM(ros::Time::now() - tfs.header.stamp << "s old\n" <<  tfs);
    } catch (tf2::TransformException& ex) {
      ROS_WARN_STREAM_THROTTLE(2.0, ex.what());
    }
  }

  return 0;
}
