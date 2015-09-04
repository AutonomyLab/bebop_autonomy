#ifndef BEBOP_DRIVER_NODELET_H
#define BEBOP_DRIVER_NODELET_H

#include <string>

#include <ros/timer.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include "bebop_autonomy/bebop.h"

namespace bebop_autonomy
{

namespace util
{
  static const double eps = 1.0e-6;

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

  inline bool CompareTwists(const geometry_msgs::Twist& lhs, const geometry_msgs::Twist& rhs)
  {
    return (fabs(lhs.linear.x - rhs.linear.x) < eps) &&
        (fabs(lhs.linear.y - rhs.linear.y) < eps) &&
        (fabs(lhs.linear.z - rhs.linear.z) < eps) &&
        (fabs(lhs.angular.x - rhs.angular.x) < eps) &&
        (fabs(lhs.angular.y - rhs.angular.y) < eps) &&
        (fabs(lhs.angular.z - rhs.angular.z) < eps);
  }

  const static ros::console::levels::Level arsal_level_to_ros[ARSAL_PRINT_MAX] = {ros::console::levels::Fatal,
                                                                                  ros::console::levels::Error,
                                                                                  ros::console::levels::Warn,
                                                                                  ros::console::levels::Info,
                                                                                  ros::console::levels::Debug,
                                                                                  ros::console::levels::Debug};

  int BebopPrintToROSLogCB (eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va);
}  // namespace util

// Forward decl
class BebopArdrone3Config;
class Bebop;

class BebopDriverNodelet : public nodelet::Nodelet
{
private:
  boost::shared_ptr<bebop_autonomy::Bebop> bebop_ptr_;
  boost::shared_ptr<boost::thread> mainloop_thread_ptr_;

  geometry_msgs::Twist bebop_twist;
  geometry_msgs::Twist prev_bebop_twist;
  geometry_msgs::Twist camera_twist;
  geometry_msgs::Twist prev_camera_twist;

  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber camera_move_sub_;
  ros::Subscriber takeoff_sub_;
  ros::Subscriber land_sub_;
  ros::Subscriber reset_sub_;

  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_ptr_;
  boost::shared_ptr<image_transport::ImageTransport> image_transport_ptr_;
  image_transport::CameraPublisher image_transport_pub_;

  sensor_msgs::CameraInfoPtr camera_info_msg_ptr_;

  // Dynamic Reconfigure
  boost::shared_ptr<dynamic_reconfigure::Server<bebop_autonomy::BebopArdrone3Config> > dynr_serv_ptr_;

  // Params (not dynamically reconfigurable, persistent)
  std::string param_frame_id_;

  // This runs in its own context
  void CameraPublisherThread();

//private:
//  volatile bool running_;

  void CmdVelCallback(const geometry_msgs::TwistConstPtr& twist);
  void CameraMoveCallback(const geometry_msgs::TwistConstPtr& twist);
  void TakeoffCallback(const std_msgs::EmptyConstPtr& empty);
  void LandCallback(const std_msgs::EmptyConstPtr& empty);
  void EmergencyCallback(const std_msgs::EmptyConstPtr& empty);

  void ParamCallback(bebop_autonomy::BebopArdrone3Config &config, uint32_t level);

public:
  BebopDriverNodelet();
  ~BebopDriverNodelet();

  virtual void onInit();

};

}  // namespace bebop_autonomy

#endif  // BEBOP_DRIVER_NODELET_H
