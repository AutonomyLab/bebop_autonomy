#include <string>

#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bebop_driver_node");

  nodelet::Loader nll;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nl_name = ros::this_node::getName();
  nll.load(nl_name, "bebop_autonomy/BebopDriverNodelet", remap, nargv);

  ROS_INFO("Hello!");
  ros::spin();
  return 0;
}
