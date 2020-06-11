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
                                               std::string runningType,
                                               std::string actuatorMode)
    : RobotBridge(robot_ctrl) {
  _runningType = runningType;
  _actuatorMode = actuatorMode;
  std::string jointName[12] = {
      "abduct_fl", "thigh_fl", "knee_fl", "abduct_hl", "thigh_hl", "knee_hl",
      "abduct_fr", "thigh_fr", "knee_fr", "abduct_hr", "thigh_hr", "knee_hr"};
  _setJsMsg.name.resize(12);
  for (int i = 0; i < 12; i++) {
    _setJsMsg.name[i] = jointName[i];
  }
  jsPub = n.advertise<sensor_msgs::JointState>("/set_js", 10);
  jsSub = n.subscribe("/get_js", 10, &MiniCheetahRobotBridge::SubJS, this);
  imuBodySub =
      n.subscribe("/imu_body", 10, &MiniCheetahRobotBridge::SubImuBody, this);
  cmdVelSub =
      n.subscribe("/cmd_vel", 10, &MiniCheetahRobotBridge::SubCmdVel, this);
  robotCtrlMode = n.advertiseService(
      "ctrl_mode", &MiniCheetahRobotBridge::ServiceCtrlMode, this);
  robotGaitType = n.advertiseService(
      "gait_type", &MiniCheetahRobotBridge::ServiceGaitType, this);
  jointCtrlMode = n.serviceClient<quadruped_robot::QuadrupedCmd>("set_jm");
}

void MiniCheetahRobotBridge::SubJS(const sensor_msgs::JointState &msg) {
  for (int i = 0; i < 12; i++) {
    _robotData.getJointPos[i] = msg.position[i];
    _robotData.getJointVel[i] = msg.velocity[i];
  }
}

void MiniCheetahRobotBridge::SubImuBody(const sensor_msgs::Imu &msg) {
  _vectorNavData.accelerometer(0, 0) = msg.linear_acceleration.x;
  _vectorNavData.accelerometer(1, 0) = msg.linear_acceleration.y;
  _vectorNavData.accelerometer(2, 0) = msg.linear_acceleration.z;

  _vectorNavData.quat(0, 0) = msg.orientation.x;
  _vectorNavData.quat(1, 0) = msg.orientation.y;
  _vectorNavData.quat(2, 0) = msg.orientation.z;
  _vectorNavData.quat(3, 0) = msg.orientation.w;

  _vectorNavData.gyro(0, 0) = msg.angular_velocity.x;
  _vectorNavData.gyro(1, 0) = msg.angular_velocity.y;
  _vectorNavData.gyro(2, 0) = msg.angular_velocity.z;
}

void MiniCheetahRobotBridge::SubCmdVel(const geometry_msgs::Twist &msg) {
  _gamepadCommand.leftStickAnalog(0, 0) = -msg.linear.y;
  _gamepadCommand.leftStickAnalog(1, 0) = msg.linear.x;

  _gamepadCommand.rightStickAnalog(0, 0) = msg.angular.x;
  _gamepadCommand.rightStickAnalog(1, 0) = msg.angular.y;
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

bool MiniCheetahRobotBridge::GetParmFromFile() {
  std::string packagePath = ros::package::getPath("quadruped_robot");
  try {
    _robotParams.initializeFromYamlFile(packagePath +
                                        "/config/mini-cheetah-defaults.yaml");
  } catch (std::exception &e) {
    ROS_ERROR("Failed to initialize robot parameters from yaml file: %s",
              e.what());
    exit(1);
  }
  if (!_robotParams.isFullyInitialized()) {
    ROS_ERROR("Failed to initialize all robot parameters");
    exit(1);
  }
  try {
    _userControlParameters->initializeFromYamlFile(
        packagePath + "/config/mc-mit-ctrl-user-parameters.yaml");
  } catch (std::exception &e) {
    ROS_ERROR("Failed to initialize user parameters from yaml file: %s",
              e.what());
    exit(1);
  }
  if (!_userControlParameters->isFullyInitialized()) {
    ROS_ERROR("Failed to initialize all user parameters");
    exit(1);
  }
  ROS_WARN("Got all parameters, starting up!");
  return true;
}

/*!
 * Initialize quadruped robot
 */
bool MiniCheetahRobotBridge::InitRobot() {
  if (_runningType == "sim") {
    // set to profile position mode
    _setJm.request.cmd = 1;
    if (jointCtrlMode.call(_setJm)) {
      ROS_WARN("set JM to profile position mode");
      usleep(100 * 1000);
    } else {
      ROS_ERROR("Failed to call Jm while setting profile position mode");
      return false;
    }

    // set to init pose in profile position mode
    _setJsMsg.position.resize(12);
    _setJsMsg.header.stamp = ros::Time::now();
    for (int i = 0; i < 12; i++) {
      _setJsMsg.position[i] = _initRobotJointPos[i];
    }
    jsPub.publish(_setJsMsg);
    _setJsMsg.position.clear();
  } else if (_runningType == "real") {
    // derektodo: sdk set init data
  } else {
    ROS_ERROR("err running type when setting data");
    return false;
  }

  // set to stand up ctrl mode in controller
  ControlParameter &param = _robotParams.collection.lookup("control_mode");
  ControlParameterValue v;
  v.d = 1.0;  // STAND_UP
  param.set(v, ControlParameterValueKind::DOUBLE);

  // get init sensor data
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();
  return true;
}

/*!
 * Main method for quadruped robot
 */
void MiniCheetahRobotBridge::Run() {
  // get parm from file
  if (!GetParmFromFile()) {
    ROS_ERROR("GetParmFromFile failed");
    exit(1);
  }

  // init robot
  if (!InitRobot()) {
    ROS_ERROR("init robot failed");
    exit(1);
  }
  ROS_WARN("init robot successful");

  // connect robotRunner
  _robotRunner = new RobotRunner(_controller, &taskManager,
                                 _robotParams.controller_dt, "robot-control");
  _robotRunner->robotType = RobotType::MINI_CHEETAH;
  _robotRunner->driverCommand = &_gamepadCommand;
  _robotRunner->vectorNavData = &_vectorNavData;
  _robotRunner->controlParameters = &_robotParams;
  _robotRunner->robotData = &_robotData;

  // init control thread
  statusTask.start();

  // robot controller start
  _robotRunner->start();

  // set actuator mode
  if (_runningType == "sim") {
    if (_actuatorMode == "torque") {
      _setJsMsg.position.clear();
      _setJsMsg.effort.resize(12);
      _setJm.request.cmd = 0;
    } else if (_actuatorMode == "position") {
      _setJsMsg.effort.clear();
      _setJsMsg.position.resize(12);
      _setJm.request.cmd = 1;
    } else {
      ROS_ERROR("wrong actuator mode");
      exit(1);
    }
    if (jointCtrlMode.call(_setJm)) {
      ROS_WARN("set JM to current mode");
    } else {
      ROS_ERROR("Failed to call JM while setting current mode");
      exit(1);
    }
  } else {
    // derektodo: set sdk mode
  }

  // main loop
  while (ros::ok()) {
    usleep(5 * 1000);
    for (int i = 0; i < 12; i++) {
      _robotData.setJointTau[i] *= _actuatorCompensate[i];
    }
    if (_actuatorMode == "torque") {
      if (_runningType == "sim") {
        _setJsMsg.header.stamp = ros::Time::now();
        for (int i = 0; i < 12; i++) {
          _setJsMsg.effort[i] = _robotData.setJointTau[i];
        }
        jsPub.publish(_setJsMsg);
      } else {
        // derektodo: sdk set data
      }
    } else {
      if (_runningType == "sim") {
        _setJsMsg.header.stamp = ros::Time::now();
        for (int i = 0; i < 12; i++) {
          _setJsMsg.position[i] = _robotData.setJointPos[i];
        }
        jsPub.publish(_setJsMsg);
      } else {
        // derektodo: sdk set data
      }
    }
    ros::spinOnce();
  }
  ROS_WARN("exit main loop");
}
