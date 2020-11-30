#ifndef SAFETY_CHECKER_H
#define SAFETY_CHECKER_H

#include <iostream>

#include "StateEstimatorContainer.h"
#include "LegController.h"
#include "Dynamics/Quadruped.h"

/**
 * The SafetyChecker handles the checks requested by the ControlFSM.
 */
template <typename T>
class SafetyChecker {
 public:
  // SafetyChecker(ControlFSMData<T>* dataIn) : data(dataIn){};
  SafetyChecker();

  // Pre checks to make sure controls are safe to run
  bool checkSafeOrientation(StateEstimatorContainer<float>& _stateEstimator);  // robot's orientation is safe to control

  // Post checks to make sure controls can be sent to robot
  bool checkPDesFoot(Quadruped<float>& _quadruped,
                     LegController<float>& _legController);          // desired foot position is not too far

  bool checkJointLimit(LegController<float>& _legController);

  bool checkForceFeedForward(LegController<float>& _legController);  // desired feedforward forces are not too large

 private:
};

#endif  // SAFETY_CHECKER_H