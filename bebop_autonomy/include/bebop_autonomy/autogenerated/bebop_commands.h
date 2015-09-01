#ifndef BEBOP_COMMON_COMMANDS_H
#define BEBOP_COMMON_COMMANDS_H
/*
 * bebop_common_commands.h
 * auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
 * Date: 2015-08-31 18:59:03.777083
 * Generator: generate.py @ f74e92a
 * Do not modify this file by hand. Check scripts/meta folder for generator files.
 */

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp> 

namespace bebop_autonomy
{

namespace cb
{

/* Base class for All SDK Commands */
class CommandBase
{
protected:
  eARCONTROLLER_DICTIONARY_KEY cmd_key_;
  ARCONTROLLER_DICTIONARY_ARG_t* arg;
  mutable ::boost::mutex mutex_;
  bool pub_enabled_;
  ::ros::Publisher ros_pub_;

public:
  CommandBase(eARCONTROLLER_DICTIONARY_KEY cmd_key, const bool pub_enabled = false)
    : cmd_key_(cmd_key), arg(NULL), pub_enabled_(pub_enabled)
  {}

  virtual ~CommandBase()
  {}

  inline eARCONTROLLER_DICTIONARY_KEY GetCommandKey() const {return cmd_key_;}

  virtual void Update(const ARCONTROLLER_DICTIONARY_ARG_t* arg, const ::ros::Time& t) = 0;
};

}  // namespace cb
}  // namespace bebop_autonomy

#endif  // BEBOP_COMMON_COMMANDS_H