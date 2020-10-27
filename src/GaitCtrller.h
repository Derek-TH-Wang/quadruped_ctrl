#ifndef GAIT_CTRLLER_H
#define GAIT_CTRLLER_H

#include <math.h>
#include <time.h>

#include <iostream>
#include <string>

#include "Controllers/ContactEstimator.h"
#include "Controllers/ControlFSMData.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/OrientationEstimator.h"
#include "Controllers/PositionVelocityEstimator.h"
#include "Controllers/RobotLegState.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/MiniCheetah.h"
#include "MPC_Ctrl/ConvexMPCLocomotion.h"
#include "Utilities/IMUTypes.h"
#include "calculateTool.h"

typedef struct StructPointerTest {
  double eff[12];
} StructPointerTest, *StructPointer;

class GaitCtrller {
 public:
  GaitCtrller(double freq, double* PIDParam);
  ~GaitCtrller();
  void SetIMUData(double* imuData);
  void SetLegData(double* motorData);
  void PreWork(double* imuData, double* motorData);
  void SetGaitType(int gaitType);
  void SetRobotVel(double* vel);
  void ToqueCalculator(double* imuData, double* motorData, double* effort);

 private:
  int _gaitType = 0;
  std::vector<double> _gamepadCommand;
  Vec4<float> ctrlParam;

  Quadruped<float> _quadruped;
  ConvexMPCLocomotion* convexMPC;
  LegController<float>* _legController;
  StateEstimatorContainer<float>* _stateEstimator;
  LegData _legdata;
  LegCommand legcommand;
  ControlFSMData<float> control_data;
  VectorNavData _vectorNavData;
  CheaterState<double>* cheaterState;
  StateEstimate<float> _stateEstimate;
  RobotControlParameters* controlParameters;
  DesiredStateCommand<float>* _desiredStateCommand;
};

extern "C" {

GaitCtrller* gCtrller;

// first step, init the controller
void init_controller(double freq, double PIDParam[]) {
  gCtrller = new GaitCtrller(freq, PIDParam);
}

// the kalman filter need to work second
void pre_work(double imuData[], double legData[]) {
  gCtrller->PreWork(imuData, legData);
}

// gait type can be set in any time
void set_gait_type(int gaitType) { gCtrller->SetGaitType(gaitType); }

// robot vel can be set in any time
void set_robot_vel(double vel[]) { gCtrller->SetRobotVel(vel); }

// after init controller and pre work, the mpc calculator can work
StructPointer toque_calculator(double imuData[], double motorData[]) {
  StructPointer p = (StructPointer)malloc(sizeof(StructPointerTest));
  double eff[12];
  gCtrller->ToqueCalculator(imuData, motorData, eff);
  for (int i = 0; i < 12; i++) {
    p->eff[i] = eff[i];
  }
  return p;
}
}

#endif