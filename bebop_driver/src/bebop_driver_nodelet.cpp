/**
Software License Agreement (BSD)

\file      bebop_driver_nodelet.cpp
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
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <cmath>
#include <algorithm>
#include <string>
#include <cstdio>

#include <bebop_driver/bebop_driver_nodelet.h>
#include <bebop_driver/BebopArdrone3Config.h>

PLUGINLIB_EXPORT_CLASS(bebop_driver::BebopDriverNodelet, nodelet::Nodelet)

namespace bebop_driver
{

namespace util
{

int BebopPrintToROSLogCB(eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va)
{
  const int32_t sz = vsnprintf(bebop_err_str, BEBOP_ERR_STR_SZ, format, va);
  bebop_err_str[std::min(BEBOP_ERR_STR_SZ, sz) - 1] = '\0';
  // We can't use variable names with ROS_*_NAMED macros
  static const std::string logger_name = std::string(ROSCONSOLE_NAME_PREFIX) + "." +
      ros::this_node::getName() + ".bebopsdk";
  // Use tag inline
  ROS_LOG(util::arsal_level_to_ros[level], logger_name, "[%s] %s", tag, bebop_err_str);
  return 1;
}

}  // namespace util

BebopDriverNodelet::BebopDriverNodelet()
 : bebop_ptr_(new bebop_driver::Bebop(util::BebopPrintToROSLogCB))
{
  NODELET_INFO("Nodelet Cstr");
}

void BebopDriverNodelet::onInit()
{
  // TODO: Remove this in future release
  warnedOnce = false;

  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  util::ResetTwist(bebop_twist);
  util::ResetTwist(camera_twist);
  util::ResetTwist(prev_bebop_twist);
  util::ResetTwist(prev_camera_twist);

  // Params (not dynamically reconfigurable, local)
  // TODO(mani-monaj): Wrap all calls to .param() in a function call to enable logging
  const bool param_reset_settings = private_nh.param("reset_settings", false);
  const std::string& param_camera_info_url = private_nh.param<std::string>("camera_info_url", "");
  const std::string& param_bebop_ip = private_nh.param<std::string>("bebop_ip", "192.168.42.1");

  param_frame_id_ = private_nh.param<std::string>("camera_frame_id", "camera");

  NODELET_INFO("Connecting to Bebop ...");
  try
  {
    bebop_ptr_->Connect(nh, private_nh, param_bebop_ip);

    if (param_reset_settings)
    {
      NODELET_WARN("Resetting all settings ...");
      bebop_ptr_->ResetAllSettings();
      // Wait for 5 seconds
      ros::Rate(ros::Duration(3.0)).sleep();
    }

    NODELET_INFO("Fetching all settings from the Drone ...");
    bebop_ptr_->RequestAllSettings();
    ros::Rate(ros::Duration(3.0)).sleep();
  }
  catch (const std::runtime_error& e)
  {
    NODELET_FATAL_STREAM("Init failed: " << e.what());
    // TODO(mani-monaj): Retry mechanism
    throw e;
  }

  cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &BebopDriverNodelet::CmdVelCallback, this);
  camera_move_sub_ = nh.subscribe("camera_control", 1, &BebopDriverNodelet::CameraMoveCallback, this);
  takeoff_sub_ = nh.subscribe("takeoff", 1, &BebopDriverNodelet::TakeoffCallback, this);
  land_sub_ = nh.subscribe("land", 1, &BebopDriverNodelet::LandCallback, this);
  reset_sub_ = nh.subscribe("reset", 1, &BebopDriverNodelet::EmergencyCallback, this);
  flattrim_sub_ = nh.subscribe("flattrim", 1, &BebopDriverNodelet::FlatTrimCallback, this);
  navigatehome_sub_ = nh.subscribe("navigate_home", 1, &BebopDriverNodelet::NavigateHomeCallback, this);
  animation_sub_ = nh.subscribe("flip", 1, &BebopDriverNodelet::FlipAnimationCallback, this);

  cinfo_manager_ptr_.reset(new camera_info_manager::CameraInfoManager(nh, "bebop_front", param_camera_info_url));
  image_transport_ptr_.reset(new image_transport::ImageTransport(nh));
  image_transport_pub_ = image_transport_ptr_->advertiseCamera("image_raw", 60);

  camera_info_msg_ptr_.reset(new sensor_msgs::CameraInfo());

  dynr_serv_ptr_.reset(new dynamic_reconfigure::Server<bebop_driver::BebopArdrone3Config>(private_nh));
  dynamic_reconfigure::Server<bebop_driver::BebopArdrone3Config>::CallbackType cb =
      boost::bind(&bebop_driver::BebopDriverNodelet::ParamCallback, this, _1, _2);

  dynr_serv_ptr_->setCallback(cb);

  try
  {
    NODELET_INFO("Enabling video stream ...");
    bebop_ptr_->StartStreaming();
  }
  catch (const::std::runtime_error& e)
  {
    NODELET_ERROR_STREAM("Start() failed: " << e.what());
    // TODO(mani-monaj): Retry mechanism
  }

  if (bebop_ptr_->IsStreamingStarted())
  {
    mainloop_thread_ptr_ = boost::make_shared<boost::thread>(
          boost::bind(&bebop_driver::BebopDriverNodelet::BebopDriverNodelet::CameraPublisherThread, this));
  }

  NODELET_INFO_STREAM("Nodelet lwp_id: " << util::GetLWPId());
}

BebopDriverNodelet::~BebopDriverNodelet()
{
  NODELET_INFO_STREAM("Bebop Nodelet Dstr: " << bebop_ptr_->IsConnected());
  if (mainloop_thread_ptr_)
  {
    mainloop_thread_ptr_->interrupt();
    mainloop_thread_ptr_->join();
  }
  if (bebop_ptr_->IsStreamingStarted()) bebop_ptr_->StopStreaming();
  if (bebop_ptr_->IsConnected()) bebop_ptr_->Disconnect();
}

void BebopDriverNodelet::CmdVelCallback(const geometry_msgs::TwistConstPtr& twist_ptr)
{
  try
  {
    bebop_twist = *twist_ptr;

    const bool is_bebop_twist_changed = !util::CompareTwists(bebop_twist, prev_bebop_twist);

    if (is_bebop_twist_changed)
    {
      // TODO: Remove message in future release
      if (!warnedOnce) {
        ROS_ERROR("ATTENTION: * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *");
        ROS_ERROR("ATTENTION: Bebop driver now follows right-hand convention (ie. +angular.z translates to CCW rotation). This message will be removed in a future release.");
        ROS_ERROR("ATTENTION: * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *");
        warnedOnce = true;
      }

      bebop_ptr_->Move(CLAMP(-bebop_twist.linear.y, -1.0, 1.0),
                       CLAMP(bebop_twist.linear.x, -1.0, 1.0),
                       CLAMP(bebop_twist.linear.z, -1.0, 1.0),
                       CLAMP(-bebop_twist.angular.z, -1.0, 1.0));
      prev_bebop_twist = bebop_twist;
    }
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void BebopDriverNodelet::TakeoffCallback(const std_msgs::EmptyConstPtr& empty_ptr)
{
  try
  {
    util::ResetTwist(bebop_twist);
    bebop_ptr_->Takeoff();
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void BebopDriverNodelet::LandCallback(const std_msgs::EmptyConstPtr& empty_ptr)
{
  try
  {
    util::ResetTwist(bebop_twist);
    bebop_ptr_->Land();
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

// We shoudld probably switch to sensor_msgs/JointState instead of Twist
void BebopDriverNodelet::CameraMoveCallback(const geometry_msgs::TwistConstPtr& twist_ptr)
{
  try
  {
    camera_twist = *twist_ptr;
    const bool is_camera_twist_changed = !util::CompareTwists(camera_twist, prev_camera_twist);
    if (is_camera_twist_changed)
    {
      // TODO(mani-monaj): Set |90| limit to appropriate value (|45|??)
      bebop_ptr_->MoveCamera(CLAMP(camera_twist.angular.y, -90.0, 90.0),
                             CLAMP(camera_twist.angular.z, -90.0, 90.0));
      prev_camera_twist = camera_twist;
    }
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void BebopDriverNodelet::EmergencyCallback(const std_msgs::EmptyConstPtr& empty_ptr)
{
  try
  {
    util::ResetTwist(bebop_twist);
    bebop_ptr_->Emergency();
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void BebopDriverNodelet::FlatTrimCallback(const std_msgs::EmptyConstPtr &empty_ptr)
{
  try
  {
    // TODO(mani-monaj): Check if landed
    ROS_INFO("Flat Trim");
    bebop_ptr_->FlatTrim();
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void BebopDriverNodelet::NavigateHomeCallback(const std_msgs::BoolConstPtr &start_stop_ptr)
{
  try
  {
    ROS_INFO("%sing navigate home behavior ...", start_stop_ptr->data ? "Start" : "Stopp");
    bebop_ptr_->NavigateHome(start_stop_ptr->data);
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void BebopDriverNodelet::FlipAnimationCallback(const std_msgs::UInt8ConstPtr &animid_ptr)
{
  try
  {
    // TODO(mani-monaj): Check if flying
    ROS_INFO("Performing flip animation %d ...", animid_ptr->data);
    bebop_ptr_->AnimationFlip(animid_ptr->data);
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void BebopDriverNodelet::ParamCallback(BebopArdrone3Config &config, uint32_t level)
{
  NODELET_INFO("Dynamic reconfigure callback with level: %d", level);
  bebop_ptr_->UpdateSettings(config);
}

// Runs its own context
void BebopDriverNodelet::CameraPublisherThread()
{
  uint32_t frame_w = 0;
  uint32_t frame_h = 0;
  ROS_INFO_STREAM("Camera publisher thread lwp_id: " << util::GetLWPId());

  while (!boost::this_thread::interruption_requested())
  {
    try
    {
      sensor_msgs::ImagePtr image_msg_ptr_(new sensor_msgs::Image());
      const ros::Time t_now = ros::Time::now();

      NODELET_DEBUG_STREAM("Grabbing a frame from Bebop");
      bebop_ptr_->GetFrontCameraFrame(image_msg_ptr_->data, frame_w, frame_h);

      NODELET_DEBUG_STREAM("Frame grabbed: " << frame_w << " , " << frame_h);
      camera_info_msg_ptr_.reset(new sensor_msgs::CameraInfo(cinfo_manager_ptr_->getCameraInfo()));
      camera_info_msg_ptr_->header.stamp = t_now;
      camera_info_msg_ptr_->header.frame_id = param_frame_id_;
      camera_info_msg_ptr_->width = frame_w;
      camera_info_msg_ptr_->height = frame_h;

      if (image_transport_pub_.getNumSubscribers() > 0)
      {
        image_msg_ptr_->encoding = "rgb8";
        image_msg_ptr_->is_bigendian = false;
        image_msg_ptr_->header.frame_id = param_frame_id_;
        image_msg_ptr_->header.stamp = t_now;
        image_msg_ptr_->width = frame_w;
        image_msg_ptr_->height = frame_h;
        image_msg_ptr_->step = image_msg_ptr_->width * 3;

        image_transport_pub_.publish(image_msg_ptr_, camera_info_msg_ptr_);
      }
    }
    catch (const std::runtime_error& e)
    {
      NODELET_ERROR_STREAM("[CameraPublisher] " << e.what());
    }
  }

  NODELET_INFO("Camera publisher thread died.");
}

}  // namespace bebop_driver
