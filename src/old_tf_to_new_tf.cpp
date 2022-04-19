/*
Copyright 2022 Lucas Walter

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1.
Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <string>

/* Look up a transform then create a new tf child that has the same parent-child relationship
*/
class OldTfToNewTf
{
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster br_;

  ros::Time last_lookup_time_ = ros::Time(0);
  bool last_lookup_failed_ = true;

  ros::Timer timer_;

  std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;

  double lookup_time_offset_ = 0.0;
  bool lookup_time_most_recent_ = true;
  std::string lookup_parent_ = "map";
  std::string lookup_child_ = "child";
  double broadcast_time_offset_;
  std::string broadcast_parent_ = "map";
  std::string broadcast_child_ = "child2";

  bool zero_rotation_ = false;
  bool zero_roll_ = false;
  bool zero_pitch_ = false;
  bool zero_yaw_ = false;
  bool zero_x_ = false;
  bool zero_y_ = false;
  bool zero_z_ = false;

  void update(const ros::TimerEvent& te)
  {
    auto cur_time = te.current_real;
    auto lookup_time = cur_time + ros::Duration();
    if (lookup_time_most_recent_) {
      lookup_time = ros::Time(0);
    }

    geometry_msgs::TransformStamped ts_out;
    ts_out.transform.rotation.w = 1.0;
    ts_out.header.frame_id = broadcast_parent_;
    ts_out.child_frame_id = broadcast_child_;

    geometry_msgs::TransformStamped ts_in;
    try {
      ts_in = tf_buffer_.lookupTransform(lookup_parent_, lookup_child_, lookup_time);
    } catch (tf2::TransformException& ex) {
      if (!last_lookup_failed_) {
        ROS_WARN_STREAM_THROTTLE(2.0, "lookup time: " << lookup_time.toSec());
        ROS_WARN_STREAM_THROTTLE(2.0, ex.what());
        last_lookup_failed_ = true;
      }
      return;
    }
    if (last_lookup_failed_) {
      ROS_WARN_STREAM_THROTTLE(2.0, "now looking up " << ts_in.header.frame_id << " to " << ts_in.child_frame_id);
    }
    last_lookup_failed_ = false;

    if (ts_in.header.stamp == last_lookup_time_) {
      return;
    }
    last_lookup_time_ = ts_in.header.stamp;

    ts_out.transform = ts_in.transform;
    ts_out.header.stamp = ts_in.header.stamp + ros::Duration(broadcast_time_offset_);

    if (zero_rotation_) {
      ts_out.transform.rotation.x = 0.0;
      ts_out.transform.rotation.y = 0.0;
      ts_out.transform.rotation.z = 0.0;
      ts_out.transform.rotation.w = 1.0;
    } else if (zero_roll_ || zero_pitch_ || zero_yaw_) {
      tf2::Quaternion quat;
      tf2::fromMsg(ts_out.transform.rotation, quat);
      tf2::Matrix3x3 rot(quat);
      double roll, pitch, yaw;
      rot.getRPY(roll, pitch, yaw);
      if (zero_roll_) {
        roll = 0.0;
      }
      if (zero_pitch_) {
        pitch = 0.0;
      }
      if (zero_yaw_) {
        yaw = 0.0;
      }
      quat.setRPY(roll, pitch, yaw);
      ts_out.transform.rotation = tf2::toMsg(quat);
    }

    if (zero_x_) {
      ts_out.transform.translation.x = 0.0;
    }
    if (zero_y_) {
      ts_out.transform.translation.y = 0.0;
    }
    if (zero_z_) {
      ts_out.transform.translation.z = 0.0;
    }

    ROS_DEBUG_STREAM_ONCE(ts_out);

    br_.sendTransform(ts_out);
  }

public:
  explicit OldTfToNewTf(const double cache_time) :
    tf_buffer_(ros::Duration(cache_time)),
    tf_listener_(tf_buffer_)
  {
    double update_rate = 20.0;
    ros::param::get("~update_rate", update_rate);

    ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>();
    ddr_->registerVariable<double>("lookup_time_offset", &lookup_time_offset_,
                                   "offset the lookup time", -10.0, 10.0);
    ddr_->registerVariable<bool>("lookup_time_most_recent", &lookup_time_most_recent_,
                                 "use the most recent tf", false, true);
    ddr_->registerVariable<std::string>("lookup_parent", &lookup_parent_, std::string("lookup parent"));
    ddr_->registerVariable<std::string>("lookup_child", &lookup_child_, std::string("lookup child"));
    ddr_->registerVariable<double>("broadcast_time_offset", &broadcast_time_offset_,
                                   "offset the broadcast time", -10.0, 10.0);
    ddr_->registerVariable<std::string>("broadcast_parent", &broadcast_parent_, std::string("broadcast parent"));
    ddr_->registerVariable<std::string>("broadcast_child", &broadcast_child_, std::string("broadcast child"));
    // zero_rotation is same as zero_roll/pitch/yaw all combined
    ddr_->registerVariable<bool>("zero_rotation", &zero_rotation_, "zero out rotation", false, true);
    ddr_->registerVariable<bool>("zero_roll", &zero_roll_, "zero out roll", false, true);
    ddr_->registerVariable<bool>("zero_pitch", &zero_pitch_, "zero out pitch", false, true);
    ddr_->registerVariable<bool>("zero_yaw", &zero_yaw_, "zero out yaw", false, true);
    ddr_->registerVariable<bool>("zero_x", &zero_x_, "zero out x", false, true);
    ddr_->registerVariable<bool>("zero_y", &zero_y_, "zero out y", false, true);
    ddr_->registerVariable<bool>("zero_z", &zero_z_, "zero out z", false, true);
    ddr_->publishServicesTopics();

    // collect some transforms
    ros::Duration(1.0).sleep();

    timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate), &OldTfToNewTf::update, this);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "old_tf_to_new_tf");
  float cache_time = 30.0;
  ros::param::get("~cache_time", cache_time);
  OldTfToNewTf old_tf_to_new_tf(cache_time);
  ros::spin();
}
