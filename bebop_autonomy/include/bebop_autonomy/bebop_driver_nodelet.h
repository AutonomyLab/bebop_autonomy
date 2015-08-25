#ifndef BEBOP_DRIVER_NODELET_H
#define BEBOP_DRIVER_NODELET_H

#include <string>

#include <ros/timer.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include <boost/shared_ptr.hpp>

#include "bebop_autonomy/bebop.h"

namespace bebop_autonomy
{

namespace util
{
  static const double eps = 1.0e-6;

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
}  // namespace util

class BebopDriverNodelet : public nodelet::Nodelet
{
private:
  bebop_autonomy::Bebop bebop_;

  geometry_msgs::Twist bebop_twist;
  geometry_msgs::Twist prev_bebop_twist;
  geometry_msgs::Twist camera_twist;
  geometry_msgs::Twist prev_camera_twist;

  bool do_takeoff;
  bool do_land;
  bool do_emergency;

  ros::Timer timer_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber camera_move_sub_;
  ros::Subscriber takeoff_sub_;
  ros::Subscriber land_sub_;
  ros::Subscriber reset_sub_;

  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_ptr_;
  boost::shared_ptr<image_transport::ImageTransport> image_transport_ptr_;
  image_transport::CameraPublisher image_transport_pub_;

  sensor_msgs::CameraInfoPtr camera_info_msg_ptr_;
  sensor_msgs::ImagePtr image_msg_ptr_;

  ros::Publisher dummy_pub_;

  // Params
  std::string frame_id_;

  void PublishVideo();
  void TimerCallback(const ros::TimerEvent& event);
//private:
//  volatile bool running_;

  void CmdVelCallback(const geometry_msgs::TwistConstPtr& twist);
  void CameraMoveCallback(const geometry_msgs::TwistConstPtr& twist);
  void TakeoffCallback(const std_msgs::EmptyConstPtr& empty);
  void LandCallback(const std_msgs::EmptyConstPtr& empty);
  void EmergencyCallback(const std_msgs::EmptyConstPtr& empty);

public:
  BebopDriverNodelet();
  ~BebopDriverNodelet();

  virtual void onInit();

};

}  // namespace bebop_autonomy

#endif  // BEBOP_DRIVER_NODELET_H
