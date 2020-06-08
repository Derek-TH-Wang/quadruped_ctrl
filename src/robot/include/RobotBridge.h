/*!
 * @file RobotBridge.h
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */

#ifndef PROJECT_ROBOTBRIDGE_H
#define PROJECT_ROBOTBRIDGE_H

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

#include <string>

#include "RobotRunner.h"
#include "Utilities/PeriodicTask.h"
#include "quadruped_robot/QuadrupedCmd.h"

/*!
 * Interface between robot and hardware
 */
class RobotBridge {
 public:
  RobotBridge(RobotController* robot_ctrl) : statusTask(&taskManager, 0.5f) {
    _controller = robot_ctrl;
    _userControlParameters = robot_ctrl->getUserControlParameters();
  }
  ~RobotBridge() { delete _robotRunner; }

 protected:
  PeriodicTaskManager taskManager;
  PrintTaskStatus statusTask;
  GamepadCommand _gamepadCommand;

  bool _firstRun = true;
  RobotRunner* _robotRunner = nullptr;
  RobotControlParameters _robotParams;
  RobotController* _controller = nullptr;
  ControlParameters* _userControlParameters = nullptr;
};

/*!
 * Interface between robot and hardware/simulator specialized for Mini Cheetah
 */
class MiniCheetahRobotBridge : public RobotBridge {
 public:
  MiniCheetahRobotBridge(RobotController* rc, std::string runningType);
  void initRobot();
  void run();

 private:
  VectorNavData _vectorNavData;
  std::string _runningType = "sim";

  ros::NodeHandle n;
  ros::Subscriber imuBodySub;
  ros::Subscriber cmdVelSub;
  ros::ServiceServer ctrlMode;
  ros::ServiceServer gaitType;
  void SubImuBody(const sensor_msgs::Imu& msg);
  void SubCmdVel(const geometry_msgs::Twist& msg);
  bool ServiceCtrlMode(quadruped_robot::QuadrupedCmd::Request& req,
                       quadruped_robot::QuadrupedCmd::Response& res);
  bool ServiceGaitType(quadruped_robot::QuadrupedCmd::Request& req,
                       quadruped_robot::QuadrupedCmd::Response& res);
};

#endif
