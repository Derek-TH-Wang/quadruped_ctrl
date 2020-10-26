#include "GaitCtrller.h"

GaitCtrller::GaitCtrller(double freq, double* PIDParam) {
  for (int i = 0; i < 4; i++) {
    ctrlParam(i) = PIDParam[i];
  }
  _gamepadCommand.resize(4);
  FloatingBaseModel<float> _model;
  convexMPC = new ConvexMPCLocomotion(1.0 / freq, 13);

  _quadruped = buildMiniCheetah<float>();
  _model = _quadruped.buildModel();

  _legController = new LegController<float>(_quadruped);

  _stateEstimator = new StateEstimatorContainer<float>(
      cheaterState, &_vectorNavData, _legController->datas, &_stateEstimate,
      controlParameters);

  // reset the state estimator
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);

  //源代码中中对姿态和位置速度的估计，添加到状态估计器中
  _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();

  _desiredStateCommand = new DesiredStateCommand<float>(1.0 / freq);

  // for (int i = 0; i < 100;
  //      i++) {  // init state estimator data because of the filter
  //   _stateEstimator->run();

  //   // Update the data from the robot
  //   _legController->updateData(&_legdata);
  // }
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

void GaitCtrller::SetRobotVel(double* vel) {
  if (abs(vel[0]) < 0.1) {
    _gamepadCommand[0] = 0.0;
  } else {
    _gamepadCommand[0] = vel[0] * 1.5;
  }

  if (abs(vel[1]) < 0.1) {
    _gamepadCommand[1] = 0.0;
  } else {
    _gamepadCommand[1] = vel[1] * 0.5;
  }

  if (abs(vel[2]) < 0.1) {
    _gamepadCommand[2] = 0.0;
  } else {
    _gamepadCommand[2] = vel[2] * 2.0;
  }
  std::cout << "set vel to: " << _gamepadCommand[0] << " " << _gamepadCommand[1]
            << " " << _gamepadCommand[2] << std::endl;
}

void GaitCtrller::ToqueCalculator(double* imuData, double* motorData,
                                  double* effort) {
  // std::vector<double> effort;
  // double effort[12];
  // set imu data
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
  // std::cout << "imu = " << std::endl;
  // for (int i = 0; i < 10; i++) {
  //   std::cout << imuData[i] << " ";
  // }
  // std::cout << std::endl;
  // set motor data
  for (int i = 0; i < 4; i++) {
    _legdata.q_abad[i] = motorData[i * 3];
    _legdata.q_hip[i] = motorData[i * 3 + 1];
    _legdata.q_knee[i] = motorData[i * 3 + 2];
    _legdata.qd_abad[i] = motorData[12 + i * 3];
    _legdata.qd_hip[i] = motorData[12 + i * 3 + 1];
    _legdata.qd_knee[i] = motorData[12 + i * 3 + 2];
  }

  // state estimator
  _stateEstimator->run();

  // Update the data from the robot
  _legController->updateData(&_legdata);

  // Setup the leg controller for a new iteration
  _legController->zeroCommand();
  _legController->setEnabled(true);
  _legController->setMaxTorqueCheetah3(208.5);

  // Find the desired state trajectory
  _desiredStateCommand->convertToStateCommands(_gamepadCommand);

  convexMPC->run(_quadruped, *_legController, *_stateEstimator,
                 *_desiredStateCommand, _gamepadCommand, _gaitType);

  _legController->updateCommand(&legcommand, ctrlParam);

  // effort.resize(12);
  for (int i = 0; i < 4; i++) {
    effort[i * 3] = legcommand.tau_abad_ff[i];
    effort[i * 3 + 1] = legcommand.tau_hip_ff[i];
    effort[i * 3 + 2] = legcommand.tau_knee_ff[i];
  }

  // return effort;
}
