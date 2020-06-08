/*!
 * @file RobotBridge.cpp
 * @brief Interface between robot code and robot hardware/simulator
 *
 * This class initializes the hardware/simulator of both robots and allows the
 * robot controller to access it
 */

#include "RobotBridge.h"

#include <ros/package.h>
#include <ros/ros.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <thread>

#include "Utilities/Utilities_print.h"

MiniCheetahRobotBridge::MiniCheetahRobotBridge(RobotController *robot_ctrl,
                                               std::string runningType)
    : RobotBridge(robot_ctrl) {
  _runningType = runningType;
  imuBodySub =
      n.subscribe("/imu_body", 10, &MiniCheetahRobotBridge::SubImuBody, this);
  cmdVelSub =
      n.subscribe("/cmd_vel", 10, &MiniCheetahRobotBridge::SubCmdVel, this);
  ctrlMode = n.advertiseService("ctrl_mode",
                                &MiniCheetahRobotBridge::ServiceCtrlMode, this);
  gaitType = n.advertiseService("gait_type",
                                &MiniCheetahRobotBridge::ServiceGaitType, this);
}

void MiniCheetahRobotBridge::SubImuBody(const sensor_msgs::Imu &msg) {
  _vectorNavData.accelerometer(0, 0) = msg.linear_acceleration.x;
  _vectorNavData.accelerometer(1, 0) = msg.linear_acceleration.y;
  _vectorNavData.accelerometer(2, 0) = msg.linear_acceleration.z;

  _vectorNavData.quat(0, 0) = msg.orientation.w;
  _vectorNavData.quat(1, 0) = msg.orientation.x;
  _vectorNavData.quat(2, 0) = msg.orientation.y;
  _vectorNavData.quat(3, 0) = msg.orientation.z;

  _vectorNavData.gyro(0, 0) = msg.angular_velocity.x;
  _vectorNavData.gyro(1, 0) = msg.angular_velocity.y;
  _vectorNavData.gyro(2, 0) = msg.angular_velocity.z;
}

void MiniCheetahRobotBridge::SubCmdVel(const geometry_msgs::Twist &msg) {
  _gamepadCommand.leftStickAnalog(0, 0) = -msg.linear.y;
  _gamepadCommand.leftStickAnalog(1, 0) = msg.linear.x;

  _gamepadCommand.rightStickAnalog(0, 0) = -msg.angular.y;
  _gamepadCommand.rightStickAnalog(1, 0) = msg.angular.x;
}

bool MiniCheetahRobotBridge::ServiceCtrlMode(
    quadruped_robot::QuadrupedCmd::Request &req,
    quadruped_robot::QuadrupedCmd::Response &res) {
  // PASSIVE 0
  // STAND_UP 1
  // BALANCE_STAND 3
  // LOCOMOTION 4
  // RECOVERY_STAND 6
  // BACKFLIP 9
  // FRONTJUMP 11
  if (req.cmd < 0 || req.cmd == 2 || req.cmd == 5 || req.cmd == 7 ||
      req.cmd == 8 || req.cmd == 10 || req.cmd > 11) {
    res.result = -1;
    res.description = "wrong input ctrl mode cmd";
    ROS_ERROR("wrong input ctrl mode cmd");
    return true;
  }
  ControlParameter &param = _robotParams.collection.lookup("control_mode");
  ControlParameterValue v;
  v.d = (double)req.cmd;
  param.set(v, ControlParameterValueKind::DOUBLE);
  res.result = 0;
  res.description = "ctrl mode cmd set successful";
  ROS_WARN("ctrl mode cmd set successful");
  return true;
}

bool MiniCheetahRobotBridge::ServiceGaitType(
    quadruped_robot::QuadrupedCmd::Request &req,
    quadruped_robot::QuadrupedCmd::Response &res) {
  // 0, // trot
  // 1, // bounding
  // 2, // pronking
  // 3, // gallop
  // 5, // trot run
  // 6, // walk};
  // 7, // walk2?
  // 8, // pace
  if (req.cmd < 0 || req.cmd == 4 || req.cmd > 8) {
    res.result = -1;
    res.description = "wrong input gait type cmd";
    ROS_ERROR("wrong input gait type cmd");
    return true;
  }
  ControlParameter &param =
      _userControlParameters->collection.lookup("cmpc_gait");
  ControlParameterValue v;
  v.d = (double)req.cmd;
  param.set(v, ControlParameterValueKind::DOUBLE);
  res.result = 0;
  res.description = "gait type cmd set successful";
  ROS_WARN("gait type cmd set successful");
  return true;
}

/*!
 * Main method for Mini Cheetah
 */
void MiniCheetahRobotBridge::run() {
  std::string packagePath = ros::package::getPath("quadruped_robot");
  initRobot();

  printf("[Hardware Bridge] Loading parameters from file...\n");
  try {
    _robotParams.initializeFromYamlFile(packagePath +
                                        "/config/mini-cheetah-defaults.yaml");
  } catch (std::exception &e) {
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
    } catch (std::exception &e) {
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

  _robotRunner =
      new RobotRunner(_controller, &taskManager, _robotParams.controller_dt,
                      "robot-control", _runningType);
  _robotRunner->robotType = RobotType::MINI_CHEETAH;
  _robotRunner->driverCommand = &_gamepadCommand;
  _robotRunner->vectorNavData = &_vectorNavData;
  _robotRunner->controlParameters = &_robotParams;

  _firstRun = false;

  // init control thread
  statusTask.start();

  // robot controller start
  _robotRunner->start();

  while (ros::ok()) {
    usleep(5 * 1000);
    ros::spinOnce();
  }
}

/*!
 * Initialize Mini Cheetah specific hardware
 */
void MiniCheetahRobotBridge::initRobot() {
  _vectorNavData.quat << 1, 0, 0, 0;
  // derektodo: real robot: init sdk, simulator: waitForMessage jointstates rostpic
}
