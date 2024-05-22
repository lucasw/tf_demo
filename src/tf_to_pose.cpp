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
#include <tf_demo/copy_transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <string>

namespace tf_demo
{

/* Look up a transform then create a new tf child that has the same parent-child relationship
*/
class TfToPose
{
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher pose_pub_;

  // double publish_rate_ = 20.0;
  // ros::Timer timer_;

  std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;

  CopyTransform copy_transform_;

  bool lookup_time_most_recent_ = true;
  double lookup_time_offset_ = 0.0;

  bool update()  // const ros::TimerEvent& te)
  {
    // auto cur_time = te.current_real;
    const auto cur_time = ros::Time::now();
    auto lookup_time = cur_time + ros::Duration(lookup_time_offset_);
    if (lookup_time_most_recent_) {
      lookup_time = ros::Time(0);
    }

    copy_transform_.broadcast_parent_ = copy_transform_.lookup_parent_;
    copy_transform_.broadcast_child_ = copy_transform_.lookup_child_;

    geometry_msgs::TransformStamped ts_out;
    const bool verbose = false;
    if (!copy_transform_.copyTransform(tf_buffer_, cur_time, lookup_time, ts_out, verbose)) {
      if (verbose) {
        ROS_WARN_STREAM_THROTTLE(8.0, "couldn't copy transform");
      }
      return false;
    }

    geometry_msgs::PoseStamped ps;
    ps.header = ts_out.header;
    ps.pose.position.x = ts_out.transform.translation.x;
    ps.pose.position.y = ts_out.transform.translation.y;
    ps.pose.position.z = ts_out.transform.translation.z;
    ps.pose.orientation = ts_out.transform.rotation;

    pose_pub_.publish(ps);

    return true;
  }

public:
  explicit TfToPose(const double cache_time) :
    tf_buffer_(ros::Duration(cache_time)),
    tf_listener_(tf_buffer_)
  {
    // TODO(lucasw) move into CopyTransform, make optional
    ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>();
    ddr_->registerVariable<double>("lookup_time_offset", &lookup_time_offset_,
                                   "offset the lookup time", -10.0, 10.0);
    ddr_->registerVariable<bool>("lookup_time_most_recent", &lookup_time_most_recent_,
                                 "use the most recent tf", false, true);
    ddr_->registerVariable<std::string>("parent", &copy_transform_.lookup_parent_,
                                        std::string("lookup parent"));
    ddr_->registerVariable<std::string>("child", &copy_transform_.lookup_child_,
                                        std::string("lookup child"));
    // TODO(lucasw) support reference frame
    // ddr_->registerVariable<std::string>("reference_frame", &reference_frame_,
    //                                     std::string("reference frame"));
    ddr_->publishServicesTopics();

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("tf_pose", 4);

    // collect some transforms
    ros::Duration(1.0).sleep();

    while (ros::ok()) {
      // This polls for a newer transform than the last update
      if (update()) {
        // having just updated don't need to update again immediately
        ros::Duration(0.02).sleep();
      } else {
        // sleep but for a smaller duration and maybe new transforms will arrive
        ros::Duration(0.002).sleep();
      }
    }
  }
};
}  // namespace tf_demo

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_to_pose");
  float cache_time = 30.0;
  ros::param::get("~cache_time", cache_time);
  tf_demo::TfToPose tf_to_posef(cache_time);
  ros::spin();
}
