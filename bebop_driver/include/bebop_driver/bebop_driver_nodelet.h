/**
Software License Agreement (BSD)

\file      bebop_driver_nodelet.h
\authors   Mani Monajjemi <mmonajje@sfu.ca>
\copyright Copyright (c) 2015, Autonomy Lab (Simon Fraser University), All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef BEBOP_AUTONOMY_BEBOP_DRIVER_NODELET_H
#define BEBOP_AUTONOMY_BEBOP_DRIVER_NODELET_H

// TODO(mani-monaj): Use forward decl here if possible

#include <string>

#include <ros/timer.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include "bebop_driver/bebop.h"

#define CLAMP(x, l, h) (((x) > (h)) ? (h) : (((x) < (l)) ? (l) : (x)))

namespace bebop_driver
{

// TODO(mani-monaj): Move these to util.h
namespace util
{
  static const ros::console::levels::Level arsal_level_to_ros[ARSAL_PRINT_MAX] = {
    ros::console::levels::Fatal,
    ros::console::levels::Error,
    ros::console::levels::Warn,
    ros::console::levels::Info,
    ros::console::levels::Debug,
    ros::console::levels::Debug};

  static const double eps = 1.0e-6;
  static const double deg2rad = 3.14159265 / 180.0;

  static char bebop_err_str[BEBOP_ERR_STR_SZ];

  void ResetTwist(geometry_msgs::Twist& t)
  {
    t.linear.x = 0.0;
    t.linear.y = 0.0;
    t.linear.z = 0.0;
    t.angular.x = 0.0;
    t.angular.y = 0.0;
    t.angular.z = 0.0;
  }

  // True: Equal, false otherwise
  // TODO(mani-monaj): refactor
  inline bool CompareTwists(const geometry_msgs::Twist& lhs, const geometry_msgs::Twist& rhs)
  {
    return (fabs(lhs.linear.x - rhs.linear.x) < eps) &&
        (fabs(lhs.linear.y - rhs.linear.y) < eps) &&
        (fabs(lhs.linear.z - rhs.linear.z) < eps) &&
        (fabs(lhs.angular.x - rhs.angular.x) < eps) &&
        (fabs(lhs.angular.y - rhs.angular.y) < eps) &&
        (fabs(lhs.angular.z - rhs.angular.z) < eps);
  }

  int BebopPrintToROSLogCB(eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va);
}  // namespace util

// Forward decl
class BebopArdrone3Config;
class Bebop;

class BebopDriverNodelet : public nodelet::Nodelet
{
private:
  boost::shared_ptr<bebop_driver::Bebop> bebop_ptr_;
  boost::shared_ptr<boost::thread> camera_pub_thread_ptr_;
  boost::shared_ptr<boost::thread> aux_thread_ptr_;

  geometry_msgs::Twist prev_bebop_twist_;
  ros::Time prev_twist_stamp_;
  boost::mutex twist_mutex_;

  geometry_msgs::Twist camera_twist_;
  geometry_msgs::Twist prev_camera_twist_;

  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber camera_move_sub_;
  ros::Subscriber takeoff_sub_;
  ros::Subscriber land_sub_;
  ros::Subscriber reset_sub_;
  ros::Subscriber flattrim_sub_;
  ros::Subscriber navigatehome_sub_;
  ros::Subscriber start_autoflight_sub_;
  ros::Subscriber pause_autoflight_sub_;
  ros::Subscriber stop_autoflight_sub_;
  ros::Subscriber animation_sub_;
  ros::Subscriber snapshot_sub_;
  ros::Subscriber exposure_sub_;
  ros::Subscriber toggle_recording_sub_;

  ros::Publisher odom_pub_;
  ros::Publisher camera_joint_pub_;
  ros::Publisher gps_fix_pub_;

  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_ptr_;
  boost::shared_ptr<image_transport::ImageTransport> image_transport_ptr_;
  image_transport::CameraPublisher image_transport_pub_;

  sensor_msgs::CameraInfoPtr camera_info_msg_ptr_;

  // Dynamic Reconfigure
  boost::shared_ptr<dynamic_reconfigure::Server<bebop_driver::BebopArdrone3Config> > dynr_serv_ptr_;

  // Params (not dynamically reconfigurable, persistent)
  std::string param_camera_frame_id_;
  std::string param_odom_frame_id_;
  bool param_publish_odom_tf_;
  double param_cmd_vel_timeout_;

  // This runs in its own context
  void CameraPublisherThread();

  // Safety monitor + ROS specfic publishers (TF, Odom, etc)
  void AuxThread();

  void CmdVelCallback(const geometry_msgs::TwistConstPtr& twist_ptr);
  void CameraMoveCallback(const geometry_msgs::TwistConstPtr& twist_ptr);
  void TakeoffCallback(const std_msgs::EmptyConstPtr& empty_ptr);
  void LandCallback(const std_msgs::EmptyConstPtr& empty_ptr);
  void EmergencyCallback(const std_msgs::EmptyConstPtr& empty_ptr);
  void FlatTrimCallback(const std_msgs::EmptyConstPtr& empty_ptr);
  void NavigateHomeCallback(const std_msgs::BoolConstPtr& start_stop_ptr);
  void StartAutonomousFlightCallback(const std_msgs::StringConstPtr& file_path_ptr);
  void PauseAutonomousFlightCallback(const std_msgs::EmptyConstPtr& empty_ptr);
  void StopAutonomousFlightCallback(const std_msgs::EmptyConstPtr& empty_ptr);
  void FlipAnimationCallback(const std_msgs::UInt8ConstPtr& animid_ptr);
  void TakeSnapshotCallback(const std_msgs::EmptyConstPtr& empty_ptr);
  void SetExposureCallback(const std_msgs::Float32ConstPtr& exposure_ptr);
  void ToggleRecordingCallback(const std_msgs::BoolConstPtr& toggle_ptr);

  void ParamCallback(bebop_driver::BebopArdrone3Config &config, uint32_t level);

public:
  BebopDriverNodelet();
  virtual ~BebopDriverNodelet();

  virtual void onInit();
};
}  // namespace bebop_driver

#endif  // BEBOP_AUTONOMY_BEBOP_DRIVER_NODELET_H
