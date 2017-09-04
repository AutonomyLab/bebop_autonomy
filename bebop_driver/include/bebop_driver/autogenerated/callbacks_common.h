/**
Software License Agreement (BSD)

\file      bebop_common_commands.h
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

 * bebop_common_commands.h
 * auto-generated from https://raw.githubusercontent.com/Parrot-Developers/arsdk-xml/ab28dab91845cd36c4d7002b55f70805deaff3c8/xml/ardrone3.xml
 * Do not modify this file by hand. Check scripts/meta folder for generator files.
 */
#ifndef BEBOP_COMMON_COMMANDS_H
#define BEBOP_COMMON_COMMANDS_H

#include <string>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

namespace bebop_driver
{

// Forward decl
class BebopArdrone3Config;

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
}  // namespace bebop_driver

#endif  // BEBOP_COMMON_COMMANDS_H