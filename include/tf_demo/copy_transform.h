/*
Copyright 2022 Lucas Walter
llowing conditions are met:

1.
Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef TF_DEMO_COPY_TRANSFORM_H
#define TF_DEMO_COPY_TRANSFORM_H

#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tf_demo
{

struct CopyTransform
{
  std::string lookup_parent_ = "map";
  std::string lookup_child_;
  std::string broadcast_parent_ = "map";
  std::string broadcast_child_;

  // TODO(lucasw) make these 0.0-1.0 where 0.0 is all the source (true) and 1.0 is all the target (false)
  // the intermediate values are linear blends
  bool zero_rotation_ = false;
  bool zero_roll_ = false;
  bool zero_pitch_ = false;
  bool zero_yaw_ = false;
  bool zero_x_ = false;
  bool zero_y_ = false;
  bool zero_z_ = false;

  ros::Time last_lookup_time_;
  bool last_lookup_failed_;

  CopyTransform()
  {
  }

  bool copyTransform(const tf2::BufferCore& buffer,
      const ros::Time& cur_time,
      const ros::Time& lookup_time,
      geometry_msgs::TransformStamped& ts_out)
  {
    // TODO(lucasw) move to cpp file, make a library in this package
    ts_out = geometry_msgs::TransformStamped();
    ts_out.transform.rotation.w = 1.0;
    ts_out.header.frame_id = broadcast_parent_;
    ts_out.child_frame_id = broadcast_child_;

    geometry_msgs::TransformStamped ts_in;
    try {
      ts_in = buffer.lookupTransform(lookup_parent_, lookup_child_, lookup_time);
    } catch (tf2::TransformException& ex) {
      if (!last_lookup_failed_) {
        ROS_WARN_STREAM_THROTTLE(2.0, "lookup time: " << lookup_time.toSec());
        ROS_WARN_STREAM_THROTTLE(2.0, ex.what());
        last_lookup_failed_ = true;
      }
      return false;
    }
    const auto lookup_elapsed = (ros::Time::now() - cur_time).toSec();

    if (last_lookup_failed_) {
      ROS_WARN_STREAM_THROTTLE(2.0, "now looking up " << ts_in.header.frame_id << " to " << ts_in.child_frame_id
          << " for " << ts_out.header.frame_id << " to " << ts_out.child_frame_id);
    }
    last_lookup_failed_ = false;

    // TODO(lucasw) for some reason lookupTransform return the same time repeatedly when the frames match,
    // causing the duplicate timestamp check below to fail- so set to current time
    // rosrun tf2_tools echo.py base_link base_link or similar does show current time,
    // so it may be a roscpp/rospy inconsistency
    // TODO(lucasw) if this is detected send a static transform
    if (lookup_child_ == lookup_parent_) {
      ts_in.header.stamp = cur_time;
    }

    // don't publish same transform with same stamp
    if (ts_in.header.stamp == last_lookup_time_) {
      return false;
    }

    // TODO(lucasw) make a debug pub that does this for every lookup
    // TODO(lucasw) need to see if static
    const auto delay = (ros::Time::now() - ts_in.header.stamp).toSec();
    ROS_DEBUG_STREAM_THROTTLE(2.0, "delay " << delay << "s, " << lookup_time.toSec()
        << "s, lookup " << lookup_elapsed << "s");

    const auto delta = (ts_in.header.stamp - last_lookup_time_).toSec();
    // TODO(lucasw) this seems to happen with sim time occasionally
    if (delta < 1e-4) {
      ROS_DEBUG_STREAM_THROTTLE(4.0, "very small delta time, skipping: "
                                     << (ts_in.header.stamp - last_lookup_time_).toSec() << "s");
      return false;
    }

    last_lookup_time_ = ts_in.header.stamp;
    ts_out.header.stamp = ts_in.header.stamp;

    ts_out.transform = ts_in.transform;

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
    return true;
  }
};

}  // namespace tf_demo

#endif  // TF_DEMO_COPY_TRANSFORM_H
