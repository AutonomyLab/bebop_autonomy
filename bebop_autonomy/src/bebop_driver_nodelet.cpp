#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <cmath>
#include <algorithm>
#include <cstdio>

#include <bebop_autonomy/bebop_driver_nodelet.h>

PLUGINLIB_EXPORT_CLASS(bebop_autonomy::BebopDriverNodelet, nodelet::Nodelet)

namespace bebop_autonomy
{

namespace util
{

int BebopPrintToROSLogCB(eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va)
{
  const int32_t sz = vsnprintf(bebop_err_str, BEBOP_ERR_STR_SZ, format, va);
  bebop_err_str[std::min(BEBOP_ERR_STR_SZ, sz) - 1] = '\0';
  // We can't use variable names with ROS_*_NAMED macros
  static const std::string logger_name = std::string(ROSCONSOLE_NAME_PREFIX) + "." + ros::this_node::getName() + ".bebopsdk";
  // Use tag inline
  ROS_LOG(util::arsal_level_to_ros[level], logger_name, "[%s] %s", tag, bebop_err_str);
  return 1;
}

}  // namespace util

BebopDriverNodelet::BebopDriverNodelet()
  : bebop_(util::BebopPrintToROSLogCB)
//    : running_(false)
{
  NODELET_INFO("Nodelet Cstr");
}

void BebopDriverNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  util::ResetTwist(bebop_twist);
  util::ResetTwist(camera_twist);
  util::ResetTwist(prev_bebop_twist);
  util::ResetTwist(prev_camera_twist);
  do_takeoff = false;
  do_emergency = false;
  do_land = false;

  NODELET_INFO("Hello NL");
  try
  {
    bebop_.Connect();
  }
  catch (const std::runtime_error& e)
  {
    NODELET_ERROR_STREAM("Init failed: " << e.what());
  }

  cmd_vel_sub_ = nh.subscribe("bebop/cmd_vel", 1, &BebopDriverNodelet::CmdVelCallback, this);
  camera_move_sub_ = nh.subscribe("bebop/camera_control", 1, &BebopDriverNodelet::CameraMoveCallback, this);
  takeoff_sub_ = nh.subscribe("bebop/takeoff", 1, &BebopDriverNodelet::TakeoffCallback, this);
  land_sub_ = nh.subscribe("bebop/land", 1, &BebopDriverNodelet::LandCallback, this);
  reset_sub_ = nh.subscribe("bebop/reset", 1, &BebopDriverNodelet::EmergencyCallback, this);

  std::string camera_info_url;
  private_nh.param<std::string>("camera_info_url", camera_info_url, "");
  private_nh.param<std::string>("frame_id", frame_id_, "camera");

  cinfo_manager_ptr_.reset(new camera_info_manager::CameraInfoManager(nh, "camera", camera_info_url));
  image_transport_ptr_.reset(new image_transport::ImageTransport(nh));
  image_transport_pub_ = image_transport_ptr_->advertiseCamera("bebop/image_raw", 10);

  camera_info_msg_ptr_.reset(new sensor_msgs::CameraInfo());
  image_msg_ptr_.reset(new sensor_msgs::Image());
  image_msg_ptr_->encoding = "rgb8";
  image_msg_ptr_->is_bigendian = false;
  image_msg_ptr_->header.frame_id = frame_id_;

  dummy_pub_ = nh.advertise<geometry_msgs::Twist>("debug", 1);

  // Video @ 30fHz
  // Command sent @ 40Hz
  timer_ = nh.createTimer(ros::Duration(1.0 / 100.0), boost::bind(&BebopDriverNodelet::TimerCallback, this , _1));
  NODELET_INFO_STREAM("[THREAD] NodeletInit: " << boost::this_thread::get_id());
}

BebopDriverNodelet::~BebopDriverNodelet()
{
  NODELET_INFO_STREAM("Bebop Nodelet Dstr: : " << bebop_.IsConnected());
  if (bebop_.IsConnected() && !bebop_.Disconnect())
  {
    NODELET_ERROR_STREAM("Bebop disconnection failed.");
  }
}

void BebopDriverNodelet::PublishVideo()
{
  // TODO: Use video recv time
  camera_info_msg_ptr_->header.stamp = ros::Time::now();
  camera_info_msg_ptr_->width = bebop_.Decoder().GetFrameWidth();
  camera_info_msg_ptr_->height = bebop_.Decoder().GetFrameHeight();

  if (image_transport_pub_.getNumSubscribers() > 0)
  {
    const uint32_t num_bytes = bebop_.Decoder().GetFrameWidth() * bebop_.Decoder().GetFrameHeight() * 3;

    if (!image_msg_ptr_->data.size())
      image_msg_ptr_->data.resize(num_bytes);

    NODELET_INFO("COPY STARTED");
    bebop_.Decoder().CopyDecodedFrame(image_msg_ptr_->data);
    NODELET_INFO("COPY FINISHED");

    image_msg_ptr_->header.stamp = ros::Time::now();
    image_msg_ptr_->width = bebop_.Decoder().GetFrameWidth();
    image_msg_ptr_->height = bebop_.Decoder().GetFrameHeight();
    image_msg_ptr_->step = image_msg_ptr_->width * 3;

    image_transport_pub_.publish(image_msg_ptr_, camera_info_msg_ptr_);
  }
}

void BebopDriverNodelet::TimerCallback(const ros::TimerEvent &event)
{
  if (!bebop_.IsConnected()) return;
//  NODELET_INFO_STREAM("In timer callback, last_duration " << event.profile.last_duration.toSec());
//  NODELET_INFO_STREAM("[THREAD] Timer: " << boost::this_thread::get_id());

  try
  {
    if (bebop_.FrameAvailableFlag().Get())
    {
      NODELET_INFO("**** new frame available");
      PublishVideo();
      bebop_.FrameAvailableFlag().Set(false);
    }
    else
    {
      NODELET_INFO("No frame is available to publish");
    }

    if (do_takeoff)
    {
      bebop_.Takeoff();
      do_takeoff = false;
      return;
    }

    if (do_land)
    {
      bebop_.Land();
      do_land = false;
      return;
    }

    const bool is_bebop_twist_changed = !util::CompareTwists(bebop_twist, prev_bebop_twist);
    const bool is_camera_twist_changed = !util::CompareTwists(camera_twist, prev_camera_twist);
    const bool hover = false;
//        (fabs(bebop_twist.linear.x) < 0.001) &&
//        (fabs(bebop_twist.linear.y) < 0.001) &&
//        (fabs(bebop_twist.linear.z) < 0.001) &&
//        (fabs(bebop_twist.angular.z) < 0.001);

    if (is_bebop_twist_changed)
    {
      bebop_.Move(bebop_twist.linear.y, bebop_twist.linear.x, bebop_twist.linear.z, bebop_twist.angular.z);
      prev_bebop_twist = bebop_twist;
    }

    if (is_camera_twist_changed)
    {
      bebop_.MoveCamera(camera_twist.linear.y, camera_twist.angular.z);
      prev_camera_twist = camera_twist;
    }

    dummy_pub_.publish(bebop_twist);
  }
  catch (const std::runtime_error& e)
  {
    NODELET_ERROR("%s", e.what());
  }

}

void BebopDriverNodelet::CmdVelCallback(const geometry_msgs::TwistConstPtr& twist)
{
  NODELET_INFO_STREAM("[THREAD] CmdVel: " << boost::this_thread::get_id());
  NODELET_INFO("In cmd_vel callback");
  bebop_twist = *twist;
}

void BebopDriverNodelet::TakeoffCallback(const std_msgs::EmptyConstPtr& empty)
{
  do_takeoff = true;
}

void BebopDriverNodelet::LandCallback(const std_msgs::EmptyConstPtr& empty)
{
  do_land = true;
}

void BebopDriverNodelet::CameraMoveCallback(const geometry_msgs::TwistConstPtr& twist)
{
  camera_twist = *twist;
}

void BebopDriverNodelet::EmergencyCallback(const std_msgs::EmptyConstPtr& empty)
{
  do_emergency = true;
}

}  // namespace bebop_autonomy
