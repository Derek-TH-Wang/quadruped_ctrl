#include "GaitCtrller.h"

GaitCtrller::GaitCtrller(double freq, double* PIDParam)
  : _quadruped{ buildMiniCheetah<float>() }
  , _model{ _quadruped.buildModel() }
  , convexMPC{ std::make_unique<ConvexMPCLocomotion>(1.0 / freq, 13) }
  , _legController{ std::make_unique<LegController<float>>(_quadruped) }
  , _stateEstimator{ std::make_unique<StateEstimatorContainer<float>>(
        cheaterState.get(), &_vectorNavData, _legController->datas, &_stateEstimate,
        controlParameters.get()) }
  , _desiredStateCommand{ std::make_unique<DesiredStateCommand<float>>(1.0 / freq) }
  , safetyChecker{ std::make_unique<SafetyChecker<float>>() }
{
  for (int i = 0; i < 4; i++) {
    ctrlParam(i) = PIDParam[i];
  }
  _gamepadCommand.resize(4);

  // reset the state estimator
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);

  _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();

  std::cout << "finish init controller" << std::endl;
}

GaitCtrller::~GaitCtrller() {}

void GaitCtrller::SetIMUData(double* imuData) {
  _vectorNavData.accelerometer(0, 0) = imuData[0];
  _vectorNavData.accelerometer(1, 0) = imuData[1];
  _vectorNavData.accelerometer(2, 0) = imuData[2];
  _vectorNavData.quat(0, 0) = imuData[3];
  _vectorNavData.quat(1, 0) = imuData[4];
  _vectorNavData.quat(2, 0) = imuData[5];
  _vectorNavData.quat(3, 0) = imuData[6];
  _vectorNavData.gyro(0, 0) = imuData[7];
  _vectorNavData.gyro(1, 0) = imuData[8];
  _vectorNavData.gyro(2, 0) = imuData[9];
}

void GaitCtrller::SetLegData(double* motorData) {
  for (int i = 0; i < 4; i++) {
    _legdata.q_abad[i] = motorData[i * 3];
    _legdata.q_hip[i] = motorData[i * 3 + 1];
    _legdata.q_knee[i] = motorData[i * 3 + 2];
    _legdata.qd_abad[i] = motorData[12 + i * 3];
    _legdata.qd_hip[i] = motorData[12 + i * 3 + 1];
    _legdata.qd_knee[i] = motorData[12 + i * 3 + 2];
  }
}

void GaitCtrller::PreWork(double* imuData, double* motorData) {
  SetIMUData(imuData);
  SetLegData(motorData);
  _stateEstimator->run();
  _legController->updateData(&_legdata);
}

void GaitCtrller::SetGaitType(int gaitType) {
  _gaitType = gaitType;
  std::cout << "set gait type to: " << _gaitType << std::endl;
}

void GaitCtrller::SetRobotMode(int mode) {
  _robotMode = mode;
  std::cout << "set robot mode to: " << _robotMode << std::endl;
}

void GaitCtrller::SetRobotVel(double* vel) {
  if (abs(vel[0]) < 0.03) {
    _gamepadCommand[0] = 0.0;
  } else {
    _gamepadCommand[0] = vel[0] * 1.0;
  }

  if (abs(vel[1]) < 0.03) {
    _gamepadCommand[1] = 0.0;
  } else {
    _gamepadCommand[1] = vel[1] * 1.0;
  }

  if (abs(vel[2]) < 0.03) {
    _gamepadCommand[2] = 0.0;
  } else {
  _gamepadCommand[2] = vel[2] * 1.0;
  }
}

void GaitCtrller::TorqueCalculator(double* imuData, double* motorData,
                                   double* effort) {
  PreWork(imuData, motorData);

  // Setup the leg controller for a new iteration
  _legController->zeroCommand();
  _legController->setEnabled(true);
  _legController->setMaxTorqueCheetah3(208.5);

  // Find the desired state trajectory
  _desiredStateCommand->convertToStateCommands(_gamepadCommand);

  //safety check
  if(!safetyChecker->checkSafeOrientation(*_stateEstimator)){
    _safetyCheck = false;
    std::cout << "broken: Orientation Safety Check FAIL" << std::endl;

  }else if (!safetyChecker->checkPDesFoot(_quadruped, *_legController)) {
    _safetyCheck = false;
    std::cout << "broken: Foot Position Safety Check FAIL" << std::endl;

  }else if (!safetyChecker->checkForceFeedForward(*_legController)) {
    _safetyCheck = false;
    std::cout << "broken: Force FeedForward Safety Check FAIL" << std::endl;

  }else if (!safetyChecker->checkJointLimit(*_legController)) {
    _safetyCheck = false;
    std::cout << "broken: Joint Limit Safety Check FAIL" << std::endl;
  }

  convexMPC->run(_quadruped, *_legController, *_stateEstimator,
                 *_desiredStateCommand, _gamepadCommand, _gaitType, _robotMode);

  _legController->updateCommand(&legcommand, ctrlParam);

  if(_safetyCheck) {
    for (int i = 0; i < 4; i++) {
      effort[i * 3] = legcommand.tau_abad_ff[i];
      effort[i * 3 + 1] = legcommand.tau_hip_ff[i];
      effort[i * 3 + 2] = legcommand.tau_knee_ff[i];
    }
  } else {
    for (int i = 0; i < 4; i++) {
      effort[i * 3] = 0.0;
      effort[i * 3 + 1] = 0.0;
      effort[i * 3 + 2] = 0.0;
    }
  }

  // return effort;
}
