#ifndef BEBOP_COMMON_COMMANDS_H
#define BEBOP_COMMON_COMMANDS_H
/*
 * bebop_common_commands.h
 * auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
 * Date: 2015-09-03
 * Do not modify this file by hand. Check scripts/meta folder for generator files.
 */

#include <string>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#include "bebop_autonomy/BebopArdrone3Config.h"

namespace bebop_autonomy
{

namespace cb
{

/* Base class for All SDK Commands */
class AbstractCommand
{
protected:
  eARCONTROLLER_DICTIONARY_KEY cmd_key_;
  ARCONTROLLER_DICTIONARY_ARG_t* arg;
  mutable ::boost::mutex mutex_;

public:
  AbstractCommand(eARCONTROLLER_DICTIONARY_KEY cmd_key)
    : cmd_key_(cmd_key), arg(NULL)
  {}

  virtual ~AbstractCommand()
  {}

  inline eARCONTROLLER_DICTIONARY_KEY GetCommandKey() const {return cmd_key_;}

  virtual void Update(const ARCONTROLLER_DICTIONARY_ARG_t* arg, const ::ros::Time& t) = 0;
};

// This is not yet abstract
class AbstractState : public AbstractCommand
{
protected:
  bool pub_enabled_;
  ::ros::Publisher ros_pub_;

public:
  AbstractState(eARCONTROLLER_DICTIONARY_KEY cmd_key, const bool pub_enabled = false)
    : AbstractCommand(cmd_key), pub_enabled_(pub_enabled)
  {}

  virtual ~AbstractState()
  {}
};

class AbstractSetting: public AbstractCommand
{
protected:
  ros::NodeHandle priv_nh_;

public:
  AbstractSetting(eARCONTROLLER_DICTIONARY_KEY cmd_key, ros::NodeHandle& priv_nh)
    : AbstractCommand(cmd_key), priv_nh_(priv_nh)
  {}

  virtual ~AbstractSetting()
  {}

  virtual void UpdateBebopFromROS(const BebopArdrone3Config &config, const ARCONTROLLER_Device_t* bebop_ctrl_ptr_) = 0;

};

}  // namespace cb
}  // namespace bebop_autonomy

#endif  // BEBOP_COMMON_COMMANDS_H