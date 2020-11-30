/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 */

#include "SafetyChecker.h"

template <typename T>
SafetyChecker<T>::SafetyChecker(){

}
/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::checkSafeOrientation(StateEstimatorContainer<float>& _stateEstimator) {
  if (abs(_stateEstimator.getResult().rpy(0)) >= 0.5 ||
      abs(_stateEstimator.getResult().rpy(1)) >= 0.5) {
        printf("Orientation safety check failed!\n");
    return false;
  } else {
    return true;
  }
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::checkPDesFoot(Quadruped<float>& _quadruped, 
                                      LegController<float>& _legController) {
  // Assumed safe to start
  bool safePDesFoot = true;

  // Safety parameters
  T maxAngle = 1.0472;  // 60 degrees (should be changed)
  T maxPDes = _quadruped._maxLegLength * sin(maxAngle);

  // Check all of the legs
  for (int leg = 0; leg < 4; leg++) {
    // Keep the foot from going too far from the body in +x
    if (_legController.commands[leg].pDes(0) > maxPDes) {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << _legController.commands[leg].pDes(0)
                << " | modified: " << maxPDes << std::endl;
      _legController.commands[leg].pDes(0) = maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in -x
    if (_legController.commands[leg].pDes(0) < -maxPDes) {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << _legController.commands[leg].pDes(0)
                << " | modified: " << -maxPDes << std::endl;
      _legController.commands[leg].pDes(0) = -maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in +y
    if (_legController.commands[leg].pDes(1) > maxPDes) {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << _legController.commands[leg].pDes(1)
                << " | modified: " << maxPDes << std::endl;
      _legController.commands[leg].pDes(1) = maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in -y
    if (_legController.commands[leg].pDes(1) < -maxPDes) {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << _legController.commands[leg].pDes(1)
                << " | modified: " << -maxPDes << std::endl;
      _legController.commands[leg].pDes(1) = -maxPDes;
      safePDesFoot = false;
    }

    // Keep the leg under the motor module (don't raise above body or crash into
    // module)
    // if (_legController.commands[leg].pDes(2) >
    //     -_quadruped._maxLegLength / 4) {
    //   std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
    //             << " | coordinate: " << 2 << "\n";
    //   std::cout << "   commanded: "
    //             << _legController.commands[leg].pDes(2)
    //             << " | modified: " << -_quadruped._maxLegLength / 4
    //             << std::endl;
    //   _legController.commands[leg].pDes(2) =
    //       -_quadruped._maxLegLength / 4;
    //   safePDesFoot = false;
    // }

    // Keep the foot within the kinematic limits
    if (_legController.commands[leg].pDes(2) <
        -_quadruped._maxLegLength) {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << _legController.commands[leg].pDes(2)
                << " | modified: " << -_quadruped._maxLegLength
                << std::endl;
      _legController.commands[leg].pDes(2) =
          -_quadruped._maxLegLength;
      safePDesFoot = false;
    }
  }

  // Return true if all desired positions are safe
  return safePDesFoot;
}

/**
 * @return safeJoint true if safe desired joint limit
 */
template <typename T>
bool SafetyChecker<T>::checkJointLimit(LegController<float>& _legController){
  bool safeJoint = true;

  T max_ab_ad_angle = 1.0472;   //60 degree
  T max_hip_angle = 0.174533;   //10 degree
  T min_hip_angle = -1.22173;    //-70 degree
  T max_knee_angle = 2.79253;    //160 degree
  T min_knee_angle = -0.174533;   //-10 degree

  for (int leg = 0; leg < 4; leg++) {
    if(_legController.datas[leg].q(0) < -max_ab_ad_angle) {
      _legController.datas[leg].q(0) = -max_ab_ad_angle;
      safeJoint = false;
    }

    if(_legController.datas[leg].q(0) > max_ab_ad_angle) {
      _legController.datas[leg].q(0) = max_ab_ad_angle;
      safeJoint = false;
    }

    if(_legController.datas[leg].q(1) < min_hip_angle) {
      _legController.datas[leg].q(1) = min_hip_angle;
      safeJoint = false;
    }

    if(_legController.datas[leg].q(1) > max_hip_angle) {
      _legController.datas[leg].q(1) = max_hip_angle;
      safeJoint = false;
    }

    if(_legController.datas[leg].q(2) > max_knee_angle) {
      _legController.datas[leg].q(2) = max_knee_angle;
      safeJoint = false;
    }

    if(_legController.datas[leg].q(2) < min_knee_angle) {
      _legController.datas[leg].q(2) = min_knee_angle;
      safeJoint = false;
    }
  }

  return safeJoint;
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::checkForceFeedForward(LegController<float>& _legController) {
  // Assumed safe to start
  bool safeForceFeedForward = true;

  // Initialize maximum vertical and lateral forces
  T maxLateralForce = 0;
  T maxVerticalForce = 0;

  // Maximum force limits for each robot
  // if (data->_quadruped->_robotType == RobotType::CHEETAH_3) {
  //   maxLateralForce = 1800;
  //   maxVerticalForce = 1800;

  // } else if (data->_quadruped->_robotType == RobotType::MINI_CHEETAH) {
  maxLateralForce = 350;
  maxVerticalForce = 350;
  // }

  // Check all of the legs
  for (int leg = 0; leg < 4; leg++) {
    // Limit the lateral forces in +x body frame
    if (_legController.commands[leg].forceFeedForward(0) >
        maxLateralForce) {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << _legController.commands[leg].forceFeedForward(0)
                << " | modified: " << maxLateralForce << std::endl;
      _legController.commands[leg].forceFeedForward(0) = maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in -x body frame
    if (_legController.commands[leg].forceFeedForward(0) <
        -maxLateralForce) {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << _legController.commands[leg].forceFeedForward(0)
                << " | modified: " << -maxLateralForce << std::endl;
      _legController.commands[leg].forceFeedForward(0) =
          -maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in +y body frame
    if (_legController.commands[leg].forceFeedForward(1) >
        maxLateralForce) {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << _legController.commands[leg].forceFeedForward(1)
                << " | modified: " << maxLateralForce << std::endl;
      _legController.commands[leg].forceFeedForward(1) = maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in -y body frame
    if (_legController.commands[leg].forceFeedForward(1) <
        -maxLateralForce) {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << _legController.commands[leg].forceFeedForward(1)
                << " | modified: " << -maxLateralForce << std::endl;
      _legController.commands[leg].forceFeedForward(1) =
          -maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the vertical forces in +z body frame
    if (_legController.commands[leg].forceFeedForward(2) >
        maxVerticalForce) {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << _legController.commands[leg].forceFeedForward(2)
                << " | modified: " << -maxVerticalForce << std::endl;
      _legController.commands[leg].forceFeedForward(2) =
          maxVerticalForce;
      safeForceFeedForward = false;
    }

    // Limit the vertical forces in -z body frame
    if (_legController.commands[leg].forceFeedForward(2) <
        -maxVerticalForce) {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << _legController.commands[leg].forceFeedForward(2)
                << " | modified: " << maxVerticalForce << std::endl;
      _legController.commands[leg].forceFeedForward(2) =
          -maxVerticalForce;
      safeForceFeedForward = false;
    }
  }

  // Return true if all feed forward forces are safe
  return safeForceFeedForward;
}

// template class SafetyChecker<double>; This should be fixed... need to make
// RobotRunner a template
template class SafetyChecker<float>;
