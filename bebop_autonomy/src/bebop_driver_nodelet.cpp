#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

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

  NODELET_INFO("Trying to connect to Bebop ...");
  try
  {
    bebop_.Connect();
  }
  catch (const std::runtime_error& e)
  {
    NODELET_FATAL_STREAM("Init failed: " << e.what());
    // TODO: Retry mechanism
    throw e;
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

  mainloop_thread_ptr_.reset(new boost::thread(
                               boost::bind(&bebop_autonomy::BebopDriverNodelet::BebopDriverNodelet::CameraPublisherThread, this)));
  NODELET_INFO_STREAM("Nodelet lwp_id: " << util::GetLWPId());
}

BebopDriverNodelet::~BebopDriverNodelet()
{
  NODELET_INFO_STREAM("Bebop Nodelet Dstr: " << bebop_.IsConnected());
  if (mainloop_thread_ptr_)
  {
    mainloop_thread_ptr_->interrupt();
    mainloop_thread_ptr_->join();
  }
  if (bebop_.IsConnected() && !bebop_.Disconnect())
  {
    NODELET_ERROR_STREAM("Bebop disconnection failed.");
  }
}

void BebopDriverNodelet::CmdVelCallback(const geometry_msgs::TwistConstPtr& twist)
{
//  NODELET_INFO_STREAM("[THREAD] CmdVel: " << boost::this_thread::get_id());
//  NODELET_INFO("In cmd_vel callback");
  try
  {
    bebop_twist = *twist;

    const bool is_bebop_twist_changed = !util::CompareTwists(bebop_twist, prev_bebop_twist);

    if (is_bebop_twist_changed)
    {
      bebop_.Move(bebop_twist.linear.y, bebop_twist.linear.x, bebop_twist.linear.z, bebop_twist.angular.z);
      prev_bebop_twist = bebop_twist;
    }
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void BebopDriverNodelet::TakeoffCallback(const std_msgs::EmptyConstPtr& empty)
{
  try
  {
   bebop_.Takeoff();
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void BebopDriverNodelet::LandCallback(const std_msgs::EmptyConstPtr& empty)
{
  try
  {
    bebop_.Land();
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void BebopDriverNodelet::CameraMoveCallback(const geometry_msgs::TwistConstPtr& twist)
{
  try
  {
    camera_twist = *twist;
    const bool is_camera_twist_changed = !util::CompareTwists(camera_twist, prev_camera_twist);
    if (is_camera_twist_changed)
    {
      bebop_.MoveCamera(camera_twist.linear.y, camera_twist.angular.z);
      prev_camera_twist = camera_twist;
    }
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void BebopDriverNodelet::EmergencyCallback(const std_msgs::EmptyConstPtr& empty)
{
  do_emergency = true;
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

      NODELET_DEBUG_STREAM("Grabbing a frame from Bebop");
      bebop_.GetFrontCameraFrame(image_msg_ptr_->data, frame_w, frame_h);

      NODELET_DEBUG_STREAM("Frame grabbed: " << frame_w << " , " << frame_h);
      camera_info_msg_ptr_->header.stamp = ros::Time::now();
      camera_info_msg_ptr_->width = frame_w;
      camera_info_msg_ptr_->height = frame_h;

      if (image_transport_pub_.getNumSubscribers() > 0)
      {
        image_msg_ptr_->encoding = "rgb8";
        image_msg_ptr_->is_bigendian = false;
        image_msg_ptr_->header.frame_id = frame_id_;
        image_msg_ptr_->header.stamp = ros::Time::now();
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

}  // namespace bebop_autonomy
