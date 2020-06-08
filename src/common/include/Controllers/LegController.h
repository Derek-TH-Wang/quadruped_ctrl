/*! @file LegController.h
 *  @brief Common Leg Control Interface and Leg Control Algorithms
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards (the low
 * level leg control boards) All quantities are in the "leg frame" which has the
 * same orientation as the body frame, but is shifted so that 0,0,0 is at the
 * ab/ad pivot (the "hip frame").
 */

#ifndef PROJECT_LEGCONTROLLER_H
#define PROJECT_LEGCONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "Dynamics/Quadruped.h"
#include "cppTypes.h"

/*!
 * Data sent from the control algorithm to the legs.
 */
template <typename T>
struct LegControllerCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() { zero(); }

  void zero();

  Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;
  Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;
};

/*!
 * Data returned from the legs to the control code.
 */
template <typename T>
struct LegControllerData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData() { zero(); }

  void setQuadruped(Quadruped<T>& quad) { quadruped = &quad; }

  void zero();

  Vec3<T> q, qd, p, v;
  Mat3<T> J;
  Vec3<T> tauEstimate;
  Quadruped<T>* quadruped;
};

/*!
 * Controller for 4 legs of a quadruped.  Works for both Mini Cheetah and
 * Cheetah 3
 */
template <typename T>
class LegController {
 public:
  LegController(Quadruped<T>& quad, std::string runningType)
      : _quadruped(quad) {
    for (auto& data : datas) data.setQuadruped(_quadruped);
    _runningType = runningType;
    _getPos.resize(12);
    _getVel.resize(12);
    _setTau.resize(12);
    _setJsMsg.name.resize(12);
    _setJsMsg.position.resize(12);
    _setJsMsg.velocity.resize(12);
    _setJsMsg.effort.resize(12);
    _setJsMsg.name[0] = "abduct_fl";
    _setJsMsg.name[1] = "thigh_fl";
    _setJsMsg.name[2] = "knee_fl";
    _setJsMsg.name[3] = "abduct_hl";
    _setJsMsg.name[4] = "thigh_hl";
    _setJsMsg.name[5] = "knee_hl";
    _setJsMsg.name[6] = "abduct_fr";
    _setJsMsg.name[7] = "thigh_fr";
    _setJsMsg.name[8] = "knee_fr";
    _setJsMsg.name[9] = "abduct_hr";
    _setJsMsg.name[10] = "thigh_hr";
    _setJsMsg.name[11] = "knee_hr";

    jsPub = n.advertise<sensor_msgs::JointState>("/set_js", 10);
    jsSub = n.subscribe("/get_js", 10, &LegController::SubJS, this);
  }

  void zeroCommand();
  void edampCommand(RobotType robot, T gain);
  void getRobotData();
  void setRobotData();
  void setEnabled(bool enabled) { _legsEnabled = enabled; };

  LegControllerCommand<T> commands[4];
  LegControllerData<T> datas[4];
  Quadruped<T>& _quadruped;
  bool _legsEnabled = false;
  T _maxTorque = 0;
  bool _zeroEncoders = false;
  u32 _calibrateEncoders = 0;

 private:
  std::string _runningType = "sim";
  ros::NodeHandle n;
  ros::Publisher jsPub;
  ros::Subscriber jsSub;
  sensor_msgs::JointState _setJsMsg;
  void SubJS(const sensor_msgs::JointState& msg);
  std::vector<double> _getPos, _getVel, _setTau;
};

template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                   Vec3<T>* p, int leg);

#endif  // PROJECT_LEGCONTROLLER_H
