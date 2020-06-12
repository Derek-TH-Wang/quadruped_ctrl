/*!
 * @file RobotRunner.cpp
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#include "RobotRunner.h"

#include <ros/ros.h>
#include <unistd.h>

#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Controllers/PositionVelocityEstimator.h"
#include "Dynamics/MiniCheetah.h"
#include "ParamHandler.hpp"
#include "Utilities/Timer.h"
#include "Utilities/Utilities_print.h"

RobotRunner::RobotRunner(RobotController* robot_ctrl,
                         PeriodicTaskManager* manager, float period,
                         std::string name)
    : PeriodicTask(manager, period, name) {
  _robot_ctrl = robot_ctrl;
}

/**
 * Initializes the robot model, state estimator, leg controller,
 * robot data, and any control logic specific data.
 */
void RobotRunner::init() {
  printf("[RobotRunner] initialize\n");

  // Build the appropriate Quadruped object
  if (robotType == RobotType::MINI_CHEETAH) {
    _quadruped = buildMiniCheetah<float>();
  } else {
    // _quadruped = buildCheetah3<float>();
  }

  // Initialize the model and robot data
  _model = _quadruped.buildModel();
  _jpos_initializer =
      new JPosInitializer<float>(3., controlParameters->controller_dt);

  // Always initialize the leg controller and state entimator
  _legController = new LegController<float>(_quadruped);
  _stateEstimator = new StateEstimatorContainer<float>(  // cheaterState,
      vectorNavData, _legController->datas, &_stateEstimate, controlParameters);
  initializeStateEstimator();

  _desiredStateCommand = new DesiredStateCommand<float>(
      driverCommand, controlParameters, &_stateEstimate,
      controlParameters->controller_dt);

  // Controller initializations
  _robot_ctrl->_model = &_model;
  _robot_ctrl->_quadruped = &_quadruped;
  _robot_ctrl->_legController = _legController;
  _robot_ctrl->_stateEstimator = _stateEstimator;
  _robot_ctrl->_stateEstimate = &_stateEstimate;
  _robot_ctrl->_robotType = robotType;
  _robot_ctrl->_controlParameters = controlParameters;
  _robot_ctrl->_desiredStateCommand = _desiredStateCommand;

  _robot_ctrl->initializeController();
}

/**
 * Runs the overall robot control system by calling each of the major components
 * to run each of their respective steps.
 */
void RobotRunner::run() {
  // Run the state estimator step
  ROS_DEBUG("state estimator run");
  _stateEstimator->run();

  // Update the data from the robot
  ROS_DEBUG("setupStep");
  setupStep();

  static int count_ini(0);
  ++count_ini;
  ROS_DEBUG("count_ini = %d", count_ini);
  if (count_ini < 10) {
    _legController->setEnabled(false);
  } else if (20 < count_ini && count_ini < 30) {
    _legController->setEnabled(false);
  } else if (40 < count_ini && count_ini < 50) {
    _legController->setEnabled(false);
  } else {
    _legController->setEnabled(true);

    // Controller
    ROS_DEBUG("jpos_initializer->IsInitialized");
    if (!_jpos_initializer->IsInitialized(_legController)) {
      Mat3<float> kpMat;
      Mat3<float> kdMat;
      // Update the jpos feedback gains
      if (robotType == RobotType::MINI_CHEETAH) {
        kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
        kdMat << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
      } else if (robotType == RobotType::CHEETAH_3) {
        kpMat << 50, 0, 0, 0, 50, 0, 0, 0, 50;
        kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
      } else {
        assert(false);
      }

      for (int leg = 0; leg < 4; leg++) {
        _legController->commands[leg].kpJoint = kpMat;
        _legController->commands[leg].kdJoint = kdMat;
      }
    } else {
      // Run Control
      ROS_DEBUG("runController");
      _robot_ctrl->runController();
    }
  }
  finalizeStep();
}

/*!
 * Before running user code, setup the leg control and estimators
 */
void RobotRunner::setupStep() {
  if (robotType == RobotType::MINI_CHEETAH) {
    _legController->updateGetRobotData(robotData);
  } else {
    ROS_ERROR("err robot type when getting data");
    assert(false);
  }
  _legController->zeroCommand();
  _legController->setEnabled(true);
}

/*!
 * After the user code, send leg commands, update state estimate
 */
void RobotRunner::finalizeStep() {
  if (robotType == RobotType::MINI_CHEETAH) {
    _legController->updateSetRobotData(robotData);
    // std::cout << "update tau = ";
    // for (int i = 0; i < 12; i++) {
    //   std::cout << robotData->setJointTau[i] << " ";
    // }
    // std::cout << std::endl;
    std::cout << "update pos = ";
    for (int i = 0; i < 12; i++) {
      std::cout << robotData->setJointPos[i] << " ";
    }
    std::cout << std::endl;
  } else {
    ROS_ERROR("err robot type when setting data");
    assert(false);
  }
  _iterations++;
}

/*!
 * Reset the state estimator in the given mode.
 * @param cheaterMode
 */
void RobotRunner::initializeStateEstimator() {
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);
  _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
}

RobotRunner::~RobotRunner() {
  delete _legController;
  delete _stateEstimator;
  delete _jpos_initializer;
}

void RobotRunner::cleanup() {}
