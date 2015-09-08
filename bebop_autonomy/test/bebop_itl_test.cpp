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

#include <gtest/gtest.h>

#include <bebop_autonomy_msgs/Ardrone3PilotingStateFlyingStateChanged.h>
#include <bebop_autonomy_msgs/Ardrone3PilotingStateAttitudeChanged.h>

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
    ROS_ERROR("CALLBACK");
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
    ROS_ERROR("CSTR");
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
  ros::Publisher takeoff_pub_;
  ros::Publisher land_pub_;
  boost::shared_ptr<util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateFlyingStateChanged> > flying_state_;
  boost::shared_ptr<util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateAttitudeChanged> > attitude_state_;

  virtual void SetUp()
  {
    ros::NodeHandle priv_nh("~");

    ROS_INFO("In SetUp()");
    // Publishers
    takeoff_pub_= nh_.advertise<std_msgs::Empty>("takeoff", 1);
    land_pub_= nh_.advertise<std_msgs::Empty>("land", 1);

    // Subs
    flying_state_.reset(new util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateFlyingStateChanged>(
                          nh_, "states/ARDrone3/PilotingState/FlyingStateChanged", 10));

    attitude_state_.reset(new util::ASyncSub<bebop_autonomy_msgs::Ardrone3PilotingStateAttitudeChanged>(
                          nh_, "states/ARDrone3/PilotingState/AttitudeChanged", 10));

    spinner_ptr_.reset(new ros::AsyncSpinner(4));
    spinner_ptr_->start();
    ROS_INFO("End SetUp()");
  }

  virtual void TearDown()
  {
    ROS_INFO("In teardown()");
    std_msgs::Empty em;
    land_pub_.publish(em);
    ros::Rate(ros::Duration(2.0)).sleep();
    spinner_ptr_->stop();
    ros::shutdown();
  }
};

TEST_F(BebopInTheLoopTest, Dummy)
{
  std_msgs::Empty em;

  // Wait 10s for Bebop to come up
  ros::Time start = ros::Time::now();

  // Check attitude_state_
  while (((ros::Time::now() - start).toSec() < 10.0) && (!attitude_state_->IsActive()))
  {
    ROS_INFO("Waiting for Bebop ...");
    ros::Rate(1.0).sleep();
  }
  ASSERT_TRUE(attitude_state_->IsActive());

  ros::Rate(ros::Duration(13)).sleep();
  ROS_WARN("Taking off ...");
  takeoff_pub_.publish(em);

  start = ros::Time::now();
  while (((ros::Time::now() - start).toSec() < 5.0))
  {
    ROS_INFO("Waiting for takeoff ...");
    if ((flying_state_->IsActive()) && (
          flying_state_->GetMsgCopy().state ==
          bebop_autonomy_msgs::Ardrone3PilotingStateFlyingStateChanged::state_hovering))
      break;
    ros::Rate(1.0).sleep();
  }
  ASSERT_TRUE(flying_state_->IsActive());
  ASSERT_EQ(
        flying_state_->GetMsgCopy().state,
        bebop_autonomy_msgs::Ardrone3PilotingStateFlyingStateChanged::state_hovering);

  ROS_WARN("Landing ...");
  land_pub_.publish(em);

  start = ros::Time::now();
  while (((ros::Time::now() - start).toSec() < 5.0))
  {
    ROS_INFO("Waiting for landing ...");
    if ((flying_state_->IsActive()) && (
          flying_state_->GetMsgCopy().state ==
          bebop_autonomy_msgs::Ardrone3PilotingStateFlyingStateChanged::state_landed))
      break;
    ros::Rate(1.0).sleep();
  }
  ASSERT_TRUE(flying_state_->IsActive());
  ASSERT_EQ(
        flying_state_->GetMsgCopy().state,
        bebop_autonomy_msgs::Ardrone3PilotingStateFlyingStateChanged::state_landed);
}

}  // namespace test
}  // namespace bebop_autonomy

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bebop_itl_test");
  ROS_INFO("Hey!");
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
