/**
Software License Agreement (BSD)

\file      bebop_driver_node.cpp
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
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bebop_driver_node", ros::init_options::NoSigintHandler);
  nodelet::Loader nll;

  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  const std::string nl_name = ros::this_node::getName();
  nll.load(nl_name, "bebop_driver/BebopDriverNodelet", remap, nargv);

  const std::vector<std::string>& loaded_nodelets = nll.listLoadedNodelets();
  if (std::find(loaded_nodelets.begin(),
                loaded_nodelets.end(),
                nl_name) == loaded_nodelets.end())
  {
    // Nodelet OnInit() failed
    ROS_FATAL("bebop_driver nodelet failed to load.");
    return 1;
  }

  // It reaches here when OnInit() succeeds
  ROS_INFO("bebop_driver nodelet loaded.");
  ros::spin();
  return 0;
}
