/*!
 * @file main.cpp
 * @brief Main Function for the robot program
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include "main_helper.h"

#include <cassert>
#include <iostream>

#include "RobotBridge.h"
#include "RobotController.h"

MasterConfig gMasterConfig;


/*!
 * Setup and run the given robot controller
 */
int main_helper(int argc, char** argv, RobotController* ctrl) {
  gMasterConfig._robot = RobotType::MINI_CHEETAH;
  gMasterConfig.simulated = false;
  gMasterConfig.load_from_file = true;

  ROS_INFO("Cheetah Software, ver = 1.0");
  ROS_INFO("Quadruped:  Mini Cheetah");
  ROS_INFO("Driver: %s", gMasterConfig.simulated
                             ? "Development Simulation Driver"
                             : "Quadruped Driver");

  if (gMasterConfig._robot == RobotType::MINI_CHEETAH) {
    MiniCheetahRobotBridge hw(ctrl, gMasterConfig.load_from_file);
    hw.run();
    ROS_INFO("[Quadruped] loop run() has finished!");
  } else {
    ROS_ERROR("[ERROR] unknown robot");
    assert(false);
  }

  return 0;
}
