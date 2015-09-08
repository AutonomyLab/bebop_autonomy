/**
Software License Agreement (BSD)

\file      bebop_itl_test.cpp
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
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <angles/angles.h>

#include <gtest/gtest.h>


#include <bebop_autonomy_msgs/Ardrone3PilotingStateFlyingStateChanged.h>
#include <bebop_autonomy_msgs/Ardrone3PilotingStateAttitudeChanged.h>
#include <bebop_autonomy_msgs/Ardrone3CameraStateOrientation.h>
#include <bebop_autonomy_msgs/Ardrone3PilotingStateFlyingStateChanged.h>
#include <bebop_autonomy_msgs/Ardrone3PilotingStateSpeedChanged.h>
#include <bebop_autonomy_msgs/Ardrone3PilotingStateAltitudeChanged.h>
#include <bebop_autonomy_msgs/CommonCommonStateBatteryStateChanged.h>

#define TIMED_ASSERT(TIMEOUT, WAIT_UNTIL_TRUE, WAITING_TEXT)                      \
do                                                                                \
{                                                                                 \
  ros::Time start = ros::Time::now();                                             \
  while (((ros::Time::now() - start).toSec() < TIMEOUT) && (!(WAIT_UNTIL_TRUE)))  \
  {                                                                               \
    ROS_INFO_ONCE(WAITING_TEXT);                                                       \
    ros::Rate(5.0).sleep();                                                       \
  }                                                                               \
  ASSERT_TRUE(WAIT_UNTIL_TRUE);                                                   \
}                                                                                 \
while (0)                                                                         \

namespace bebop_autonomy
{
namespace util
{

template<typename T>
class ASyncSub
{
private:
  ros::NodeHandle nh;
  bool active_;
  ros::Time last_updated_;
  std::string topic_;
  std::size_t queue_size_;
  ros::Subscriber sub_;
  boost::shared_ptr<T const> msg_cptr_;
  mutable boost::mutex mutex_;

  void cb(const boost::shared_ptr<T const> &msg_cptr)
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    active_ = true;
    last_updated_ = ros::Time::now();
    msg_cptr_ = msg_cptr;
  }

public:
  ASyncSub(ros::NodeHandle& nh,
                    const std::string& topic,
                    const std::size_t queue_size)
    : nh(nh), active_(false), last_updated_(0), topic_(topic)
  {
    sub_ = nh.subscribe<T>(topic_, queue_size, boost::bind(&ASyncSub<T>::cb, this, _1));
  }

  T GetMsgCopy() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return *msg_cptr_;
  }

  // Not thread safe
  const boost::shared_ptr<T const>& GetMsgConstPtr() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return msg_cptr_;
  }

  // T operator ()() const {return GetMsgCopy();}

  // Not thread safe?
  const boost::shared_ptr<T const>& operator()() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return GetMsgConstPtr();
  }

  void Deactivate()
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    active_ = false;
  }

  void DeactivateIfOlderThan(const double seconds)
  {
    if (!IsActive()) return;
    if (GetFreshness().toSec() > seconds)
    {
      ROS_WARN("Information on topic (%s) is older than (%4.2lf) seconds. Deactivating.", topic_.c_str(), seconds);
      Deactivate();
    }
  }

  bool IsActive() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return active_;
  }

  const ros::Time GetLastUpdated() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return last_updated_;
  }

  const ros::Duration GetFreshness() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return ros::Time::now() - last_updated_;
  }
};

}  // namespace util

namespace test
{

class BebopInTheLoopTest: public ::testing::Test
{
protected:
  ros::NodeHandle nh_;
  boost::shared_ptr<ros::AsyncSpinner> spinner_ptr_;

  ros::Publisher land_pub_;

  std_msgs::Empty em;
  geometry_msgs::Twist tw;

  boost::shared_ptr<util::ASyncSub<sensor_msgs::Image> > image_;

//  boost::shared_ptr<util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateAttitudeChanged> > attitude_state_;

  virtual void SetUp()
  {
    ros::NodeHandle priv_nh("~");

    ROS_INFO("In SetUp()");

    // Common Publishers
    land_pub_= nh_.advertise<std_msgs::Empty>("land", 1);

    // Common Subs
    image_.reset(new util::ASyncSub<sensor_msgs::Image>(
                          nh_, "image_raw", 10));


    spinner_ptr_.reset(new ros::AsyncSpinner(4));
    spinner_ptr_->start();

    // Wait 10s for Bebop to come up
    ros::Time start = ros::Time::now();

    // Check image_ (when image_ is being published, everything else is ready)
    TIMED_ASSERT(15.0, image_->IsActive(), "Waiting for Bebop ...");
    TIMED_ASSERT(5.0, land_pub_.getNumSubscribers() > 0, "Waiting for land subscription ...");

    ROS_INFO("End SetUp()");
  }

  virtual void TearDown()
  {
    ROS_INFO("In teardown()");
    std_msgs::Empty em;
    land_pub_.publish(em);
    ros::Rate(ros::Duration(2.0)).sleep();
    spinner_ptr_->stop();
  }
};


TEST_F(BebopInTheLoopTest, TakeOffLand)
{
  boost::shared_ptr<util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateFlyingStateChanged> >
      flying_state(new util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateFlyingStateChanged>(
                      nh_, "states/ARDrone3/PilotingState/FlyingStateChanged", 10));

  boost::shared_ptr<util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateSpeedChanged> >
      speed_state(new util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateSpeedChanged>(
                      nh_, "states/ARDrone3/PilotingState/SpeedChanged", 10));

  boost::shared_ptr<util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateAltitudeChanged> >
      alt_state(new util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateAltitudeChanged>(
                      nh_, "states/ARDrone3/PilotingState/AltitudeChanged", 10));

  boost::shared_ptr<util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateAttitudeChanged> >
      att_state(new util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateAttitudeChanged>(
                      nh_, "states/ARDrone3/PilotingState/AttitudeChanged", 10));

  // The problem with the battery state is that it is only updated when its values changes,
  // this is more likely to happen when flying than sitting on the ground
  boost::shared_ptr<util::ASyncSub<bebop_autonomy_msgs::CommonCommonStateBatteryStateChanged> >
      bat_state(new util::ASyncSub<bebop_autonomy_msgs::CommonCommonStateBatteryStateChanged>(
                      nh_, "states/common/CommonState/BatteryStateChanged", 10));

  ros::Publisher takeoff_pub =  nh_.advertise<std_msgs::Empty>("takeoff", 1);
  ros::Publisher cmdvel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // Wait 5s time for connections to establish
  TIMED_ASSERT(5.0, takeoff_pub.getNumSubscribers() > 0, "Waiting for takeoff subscription ...");
  TIMED_ASSERT(5.0, cmdvel_pub.getNumSubscribers() > 0, "Waiting for cmd_vel subscription ...");

  ROS_WARN("Taking off ...");
  takeoff_pub.publish(em);
  TIMED_ASSERT(10.0,
               flying_state->IsActive() && flying_state->GetMsgCopy().state ==
                 bebop_autonomy_msgs::Ardrone3PilotingStateFlyingStateChanged::state_hovering,
               "Waiting for takeoff to finish ..."
               );

  TIMED_ASSERT(5.0, speed_state->IsActive(), "Waiting for Speed measurements ...");
  TIMED_ASSERT(5.0, alt_state->IsActive(), "Waiting for alt measurements ...");
  TIMED_ASSERT(5.0, att_state->IsActive(), "Waiting for attitude measurements ...");

  tw.linear.x = 0.5;
  tw.linear.y = 0.0;
  tw.linear.z = 0.0;
  tw.angular.z = 0.0;
  ROS_WARN("Moving forwared ...");
  cmdvel_pub.publish(tw);
  TIMED_ASSERT(2.0, speed_state->GetMsgCopy().speedX > 0.2, "Measuring speed X ...");

  tw.linear.x = -0.5;
  tw.linear.y = 0.0;
  tw.linear.z = 0.0;
  tw.angular.z = 0.0;
  ROS_WARN("Moving Backward ...");
  cmdvel_pub.publish(tw);
  TIMED_ASSERT(2.0, speed_state->GetMsgCopy().speedX < -0.2, "Measuring speed X ...");

  tw.linear.x = 0.0;
  tw.linear.y = 0.5;
  tw.linear.z = 0.0;
  tw.angular.z = 0.0;
  ROS_WARN("Moving right ...");
  cmdvel_pub.publish(tw);
  TIMED_ASSERT(2.0, speed_state->GetMsgCopy().speedY > 0.2, "Measuring speed Y ...");

  tw.linear.x = 0.0;
  tw.linear.y = -0.5;
  tw.linear.z = 0.0;
  tw.angular.z = 0.0;
  ROS_WARN("Moving left ...");
  cmdvel_pub.publish(tw);
  TIMED_ASSERT(2.0, speed_state->GetMsgCopy().speedY < -0.2, "Measuring speed Y ...");

  /* By this time, battery state must have been changed */
  TIMED_ASSERT(2.0, bat_state->IsActive(), "Measuring battery ...");
  const uint8_t bat_percent = bat_state->GetMsgCopy().percent;

  double alt_start, yaw_start;
  // Make sure altitude is fresh
  ASSERT_LT(alt_state->GetFreshness().toSec(), 0.5);
  alt_start = alt_state->GetMsgCopy().altitude;
  tw.linear.x = 0.0;
  tw.linear.y = 0.0;
  tw.linear.z = 0.2;
  tw.angular.z = 0.0;
  ROS_WARN("Going up for 0.5m ...");
  cmdvel_pub.publish(tw);
  TIMED_ASSERT(10.0, (alt_state->GetMsgCopy().altitude - alt_start) >= 0.5, "Measuring altitude ...");

  // Make sure altitude is fresh
  ASSERT_LT(alt_state->GetFreshness().toSec(), 0.5);
  alt_start = alt_state->GetMsgCopy().altitude;
  tw.linear.x = 0.0;
  tw.linear.y = 0.0;
  tw.linear.z = -0.2;
  tw.angular.z = 0.0;
  ROS_WARN("Going down for 0.5m ...");
  cmdvel_pub.publish(tw);
  TIMED_ASSERT(10.0, (alt_state->GetMsgCopy().altitude - alt_start) <= -0.5, "Measuring altitude ...");

  // Make sure alttitude is fresh
  ASSERT_LT(att_state->GetFreshness().toSec(), 0.5);
  yaw_start = att_state->GetMsgCopy().yaw;
  tw.linear.x = 0.0;
  tw.linear.y = 0.0;
  tw.linear.z = 0.0;
  tw.angular.z = 0.5;
  ROS_WARN("Rotating CW for 90 degrees ...");
  cmdvel_pub.publish(tw);
  TIMED_ASSERT(10.0,
               angles::normalize_angle(att_state->GetMsgCopy().yaw - yaw_start) >= 0.5 * 3.141596,
               "Measuring Yaw");

  // Make sure alttitude is fresh
  ASSERT_LT(att_state->GetFreshness().toSec(), 0.5);
  yaw_start = att_state->GetMsgCopy().yaw;
  tw.linear.x = 0.0;
  tw.linear.y = 0.0;
  tw.linear.z = 0.0;
  tw.angular.z = -0.5;
  ROS_WARN("Rotating CCW for 90 degrees ...");
  cmdvel_pub.publish(tw);
  TIMED_ASSERT(10.0,
               angles::normalize_angle(att_state->GetMsgCopy().yaw - yaw_start) <= -0.5 * 3.141596,
               "Measuring Yaw");

  tw.linear.x = 0.0;
  tw.linear.y = 0.0;
  tw.linear.z = 0.0;
  tw.angular.z = 0.0;
  ROS_WARN("Stop ...");
  cmdvel_pub.publish(tw);

  ROS_WARN("Landing ...");
  land_pub_.publish(em);

  TIMED_ASSERT(10.0,
               flying_state->IsActive() && flying_state->GetMsgCopy().state ==
                 bebop_autonomy_msgs::Ardrone3PilotingStateFlyingStateChanged::state_landed,
               "Waiting for land to finish..."
               );

  ASSERT_GE(bat_percent - bat_state->GetMsgCopy().percent, 0);
}

TEST_F(BebopInTheLoopTest, ImageTest)
{
  TIMED_ASSERT(2.0, image_->IsActive(), "Waiting for front video stream ...");
  sensor_msgs::Image img = image_->GetMsgCopy();

  const std::size_t sz = img.step * img.height;

  ASSERT_TRUE(img.header.frame_id.length());
  ASSERT_GT(img.width, 0);
  ASSERT_GT(img.height, 0);
  ASSERT_GT(img.header.stamp.toSec(), 0.0);
  ASSERT_GT(sz, 0);

  double sum = 0.0;
  for (std::size_t i = 0; i < sz; i++, sum += img.data[i]);

  ASSERT_GE(sum , 0.0);
}

TEST_F(BebopInTheLoopTest, CameraMoveTest)
{
  boost::shared_ptr<util::ASyncSub<bebop_autonomy_msgs::Ardrone3CameraStateOrientation> >
      camera_state(new util::ASyncSub<bebop_autonomy_msgs::Ardrone3CameraStateOrientation>(
                      nh_, "states/ARDrone3/CameraState/Orientation", 10));

  ros::Publisher twist_pub =  nh_.advertise<geometry_msgs::Twist>("camera_control", 1);
  TIMED_ASSERT(5.0, twist_pub.getNumSubscribers() > 0, "Waiting for camera control subscription ...");

  // tilt
  tw.angular.y = 13;
  // pan
  tw.angular.z = -16;

  ROS_WARN("Moving the virtual camera");
  twist_pub.publish(tw);
  ros::Rate(ros::Duration(2)).sleep();

  TIMED_ASSERT(5.0, camera_state->IsActive(), "Waiting for camera state ...");

  ASSERT_EQ(camera_state->GetMsgCopy().tilt, 13);
  ASSERT_EQ(camera_state->GetMsgCopy().pan, -16);
}

TEST_F(BebopInTheLoopTest, DISABLED_FlatTrimTest)
{
  boost::shared_ptr<util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateFlyingStateChanged> >
      flattrim_state(new util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateFlyingStateChanged>(
                      nh_, "states/ARDrone3/PilotingState/FlyingStateChanged", 10));
    ros::Publisher flattrim_pub =  nh_.advertise<std_msgs::Empty>("flattrim", 1);

    // Wait 5s time for connection to establish
    TIMED_ASSERT(5.0, flattrim_pub.getNumSubscribers() > 0, "Waiting for flattrim subscription ...");

    ROS_WARN("Flat trim ...");
    flattrim_pub.publish(em);

    TIMED_ASSERT(10.0,
                 flattrim_state->IsActive(),
                 "Waiting for flattrim ack ..."
                 );
}
}  // namespace test
}  // namespace bebop_autonomy



int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bebop_itl_test");
  ros::NodeHandle keep_me_awake;
  testing::InitGoogleTest(&argc, argv);
  const int res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
