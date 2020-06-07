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

  std::string robotSelect, runningType;
  n.getParam("/main_parm/robot_select/", robotSelect);
  n.getParam("/main_parm/running_type/", runningType);
  ROS_INFO("Quadruped:  %s", robotSelect.c_str());
  ROS_INFO("Driver: %s", runningType.c_str());

  RobotController* ctrl = new MIT_Controller();

  if (robotSelect == "MINI_CHEETAH") {
    MiniCheetahRobotBridge hw(ctrl, runningType);
    hw.run();
    ROS_INFO("[Quadruped] loop run() has finished!");
  } else {
    ROS_ERROR("unknown robot");
    return 0;
  }

  ros::spin();
  ROS_INFO("exit quadruped_ctrl node");
  return 0;
}
