/*!
 * @file RobotRunner.h
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#ifndef PROJECT_ROBOTRUNNER_H
#define PROJECT_ROBOTRUNNER_H

#include "ControlParameters/ControlParameterInterface.h"
#include "ControlParameters/RobotParameters.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/LegController.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/Quadruped.h"
#include "JPosInitializer.h"
#include "RobotController.h"
#include "Utilities/PeriodicTask.h"

class RobotRunner : public PeriodicTask {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotRunner(RobotController*, PeriodicTaskManager*, float, std::string, std::string);
  using PeriodicTask::PeriodicTask;
  void init() override;
  void run() override;
  void cleanup() override;

  // Initialize the state estimator with default no cheaterMode
  void initializeStateEstimator();
  virtual ~RobotRunner();

  RobotController* _robot_ctrl;
  RobotType robotType;
  VectorNavData* vectorNavData;
  RobotControlParameters* controlParameters;

 private:
  float _ini_yaw;

  int iter = 0;

  void setupStep();
  void finalizeStep();

  JPosInitializer<float>* _jpos_initializer;
  Quadruped<float> _quadruped;
  LegController<float>* _legController = nullptr;
  StateEstimate<float> _stateEstimate;
  StateEstimatorContainer<float>* _stateEstimator;
  bool _cheaterModeEnabled = false;
  DesiredStateCommand<float>* _desiredStateCommand;

  FloatingBaseModel<float> _model;
  u64 _iterations = 0;
  std::string _runningType = "sim";
};

#endif  // PROJECT_ROBOTRUNNER_H
