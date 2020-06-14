#include <ros/ros.h>

#include "MIT_Controller.hpp"
#include "RobotBridge.h"
#include "RobotController.h"

int main(int argc, char** argv) {
  ROS_INFO("start quadruped_ctrl node");
  ROS_INFO("Cheetah Software, Version = 1.0");

  ros::init(argc, argv, "quadruped_ctrl");
  ros::NodeHandle n;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);

  RobotController* ctrl = new MIT_Controller();

  MiniCheetahRobotBridge bridge(ctrl);
  bridge.Run();
  ROS_INFO("[Quadruped] loop run() has finished!");

  ros::spin();
  ROS_INFO("exit quadruped_ctrl node");
  return 0;
}
