/*!
 * @file RobotBridge.cpp
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */

#include "RobotBridge.h"

#include <ros/package.h>
#include <ros/ros.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <thread>

#include "Utilities/Utilities_print.h"

MiniCheetahRobotBridge::MiniCheetahRobotBridge(RobotController* robot_ctrl,
                                               std::string runningType)
    : RobotBridge(robot_ctrl) {
  _runningType = runningType;
}

/*!
 * Main method for Mini Cheetah hardware
 */
void MiniCheetahRobotBridge::run() {
  std::string packagePath = ros::package::getPath("quadruped_robot");
  initHardware();

  printf("[Hardware Bridge] Loading parameters from file...\n");
  try {
    _robotParams.initializeFromYamlFile(packagePath +
                                        "/config/mini-cheetah-defaults.yaml");
  } catch (std::exception& e) {
    printf("Failed to initialize robot parameters from yaml file: %s\n",
           e.what());
    exit(1);
  }
  if (!_robotParams.isFullyInitialized()) {
    printf("Failed to initialize all robot parameters\n");
    exit(1);
  }
  printf("Loaded robot parameters\n");
  if (_userControlParameters) {
    try {
      _userControlParameters->initializeFromYamlFile(
          packagePath + "/config/mc-mit-ctrl-user-parameters.yaml");
    } catch (std::exception& e) {
      printf("Failed to initialize user parameters from yaml file: %s\n",
             e.what());
      exit(1);
    }
    if (!_userControlParameters->isFullyInitialized()) {
      printf("Failed to initialize all user parameters\n");
      exit(1);
    }
    printf("Loaded user parameters\n");
  } else {
    printf("Did not load user parameters because there aren't any\n");
  }
  printf("[Hardware Bridge] Got all parameters, starting up!\n");

  _robotRunner = new RobotRunner(_controller, &taskManager,
                                 _robotParams.controller_dt, "robot-control");
  _robotRunner->robotType = RobotType::MINI_CHEETAH;
  // derektodo: imu callback function in this file, sensor_msgs/Imu.msg
  _robotRunner->vectorNavData = &_vectorNavData;
  _robotRunner->controlParameters = &_robotParams;

  _firstRun = false;

  // init control thread
  statusTask.start();

  // robot controller start
  _robotRunner->start();

  while(ros::ok()) {
    usleep(5 * 1000);
    ros::spinOnce();
  }
}

/*!
 * Initialize Mini Cheetah specific hardware
 */
void MiniCheetahRobotBridge::initHardware() {
  _vectorNavData.quat << 1, 0, 0, 0;
  // derektodo: init sdk
}
