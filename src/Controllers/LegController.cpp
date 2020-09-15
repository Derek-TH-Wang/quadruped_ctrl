/*! @file LegController.cpp
 *  @brief Common Leg Control Interface
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#include "Controllers/LegController.h"
#include <fstream> 
#include <sys/time.h>
/*!
 * Zero the leg command so the leg will not output torque
 */
template <typename T>
void LegControllerCommand<T>::zero() {
  tauFeedForward = Vec3<T>::Zero();
  forceFeedForward = Vec3<T>::Zero();
  qDes = Vec3<T>::Zero();
  qdDes = Vec3<T>::Zero();
  pDes = Vec3<T>::Zero();
  vDes = Vec3<T>::Zero();
  kpCartesian = Mat3<T>::Zero();
  kdCartesian = Mat3<T>::Zero();
  kpJoint = Mat3<T>::Zero();
  kdJoint = Mat3<T>::Zero();
}

/*!
 * Zero the leg data
 */
template <typename T>
void LegControllerData<T>::zero() {
  q = Vec3<T>::Zero();
  qd = Vec3<T>::Zero();
  p = Vec3<T>::Zero();
  v = Vec3<T>::Zero();
  J = Mat3<T>::Zero();
  tauEstimate = Vec3<T>::Zero();
}

/*!
 * Zero all leg commands.  This should be run *before* any control code, so if
 * the control code is confused and doesn't change the leg command, the legs
 * won't remember the last command.
 */
template <typename T>
void LegController<T>::zeroCommand() {
  for (auto& cmd : commands) {
    cmd.zero();
  }
  _legsEnabled = false;
}

/*!
 * Set the leg to edamp.  This overwrites all command data and generates an
 * emergency damp command using the given gain. For the mini-cheetah, the edamp
 * gain is Nm/(rad/s), and for the Cheetah 3 it is N/m. You still must call
 * updateCommand for this command to end up in the low-level command data!
 */
template <typename T>
void LegController<T>::edampCommand(RobotType robot, T gain) {
  zeroCommand();
  if (robot == RobotType::CHEETAH_3) {
    for (int leg = 0; leg < 4; leg++) {
      for (int axis = 0; axis < 3; axis++) {
        commands[leg].kdCartesian(axis, axis) = gain;
      }
    }
  } else {  // mini-cheetah
    for (int leg = 0; leg < 4; leg++) {
      for (int axis = 0; axis < 3; axis++) {
        commands[leg].kdJoint(axis, axis) = gain;
      }
    }
  }
}

/*!
 * Update the "leg data" from a SPIne board message
 */
template <typename T>
void LegController<T>::updateData(LegData* legData) {
  for (int leg = 0; leg < 4; leg++) {
    // q: 关节角
    datas[leg].q(0) = legData->q_abad[leg];
    datas[leg].q(1) = legData->q_hip[leg];
    datas[leg].q(2) = legData->q_knee[leg];

    // if(flags < 20){
    //   std::cout << "the leg data is : " << leg << "  " << datas[leg].q << std::endl;
    // }
    

    // qd 关节角速度？
    datas[leg].qd(0) = legData->qd_abad[leg];
    datas[leg].qd(1) = legData->qd_hip[leg];
    datas[leg].qd(2) = legData->qd_knee[leg];

    // J and p 雅可比和足端位置
    computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &(datas[leg].J),
                                     &(datas[leg].p), leg);
    
    // if(flags < 10){
    // std::cout << "the leg data is : " << leg << "  " << datas[leg].p << std::endl;
    // }
    
    // v 足端速度
    datas[leg].v = datas[leg].J * datas[leg].qd;
  }
}

/*!
 * Update the "leg command" for the SPIne board message
 */
template <typename T>
void LegController<T>::updateCommand(LegCommand* legCommand) {
if(flags > 0){
  for (int leg = 0; leg < 4; leg++) {
    // tauFF 获得从控制器来的力矩
    Vec3<T> legTorque = commands[leg].tauFeedForward;

    // forceFF 获得从控制器来的力矩
    Vec3<T> footForce = commands[leg].forceFeedForward;
    // Vec3<T> footForce(0, 0, 10);

    // cartesian PD 直角坐标下pd
    footForce += 
        commands[leg].kpCartesian * (commands[leg].pDes - datas[leg].p);
    footForce +=
        commands[leg].kdCartesian * (commands[leg].vDes - datas[leg].v);

    // Torque 足力转换成力矩
    legTorque += datas[leg].J.transpose() * footForce;

    // set command: 命令设置 设置力矩
    legCommand->tau_abad_ff[leg] = legTorque(0);
    legCommand->tau_hip_ff[leg] = legTorque(1);
    legCommand->tau_knee_ff[leg] = legTorque(2);

    if(flags < 12){
      // std::cout << "the Pcommand is1[" << leg << "]: " << commands[leg].pDes << std::endl;
      // std::cout << "the Pcommand is2[" << leg << "]: " << datas[leg].p << std::endl;
      // std::cout << "the KP is [" << leg << "]: " << commands[leg].kpCartesian << std::endl;
      // std::cout << "the KD is [" << leg << "]: " << commands[leg].kdCartesian << std::endl;
      // std::cout << "the Pcommand is3[" << leg << "]: " << commands[leg].vDes << std::endl;
      // std::cout << "the Pcommand is4[" << leg << "]: " << datas[leg].v << std::endl;
      // std::cout << "tau force is :" << leg << "  " << legTorque(0) << ", " << legTorque(1) << ", " << legTorque(2) << ", " << commands[leg].forceFeedForward << std::endl;
      // std::cout << "tau force is222: " << leg << "  " << footForce << std::endl;
    }

    // printf("[%d] %7.3f %7.3f %7.3f\n", leg, legTorque(0), legTorque(1), legTorque(2));

    // if(flags < 30){
    //   std::cout << "the value is : " << leg << "  " << flags << "  " << commands[leg].forceFeedForward << std::endl;
    //   // std::cout << "tau force is :" << leg << "  " << legTorque(0) << ", " << legTorque(1) << ", " << legTorque(2) << ", " << commands[leg].forceFeedForward << std::endl;
    // }
    

    std::ofstream fp;
    fp.open("position.txt", std::ofstream::app);
    if(!fp){
      std::ofstream fpout("position.txt");
      fpout << commands[0].pDes(0) << "," << commands[0].pDes(1) << "," << commands[0].pDes(2) << "," << commands[1].pDes(0) << "," << commands[1].pDes(1) << "," << commands[1].pDes(2) << ","; 
      fp.close();
      fpout.close();
    }else{
      fp << commands[0].pDes(0) << "," << commands[0].pDes(1) << "," << commands[0].pDes(2) << "," << commands[1].pDes(0) << "," << commands[1].pDes(1) << "," << commands[1].pDes(2) << std::endl; 
      fp.close();
    }

    // joint space pd
    // joint space PD
    // Vec3<T> jointAngle;
    // jointAngle = Vec3<T>::Zero();
    // computeLegIK(_quadruped, commands[leg].pDes, &jointAngle, leg);
    // legCommand->kd_abad[leg] = 1;
    // legCommand->kd_hip[leg] = 1;
    // legCommand->kd_knee[leg] = 1;

    // legCommand->kp_abad[leg] = 10;
    // legCommand->kp_hip[leg] = 10;
    // legCommand->kp_knee[leg] = 10;

    // legCommand->q_des_abad[leg] = jointAngle(0);
    // legCommand->q_des_hip[leg] = jointAngle(1);
    // legCommand->q_des_knee[leg] = jointAngle(2);

    // // std::cout << "commands Q value is :" << jointAngle << std::endl;

    // legCommand->qd_des_abad[leg] = 0;
    // legCommand->qd_des_hip[leg] = 0;
    // legCommand->qd_des_knee[leg] = 0;

    // legCommand->tau_abad_ff[leg] = legCommand->kp_abad[leg] * (legCommand->q_des_abad[leg] - datas[leg].q(0)) + 
    //                                legCommand->kd_abad[leg] * (legCommand->qd_des_abad[leg] - datas[leg].qd(0));
    // legCommand->tau_hip_ff[leg] = legCommand->kp_hip[leg] * (legCommand->q_des_hip[leg] - datas[leg].q(1)) + 
    //                               legCommand->kd_hip[leg] * (legCommand->qd_des_hip[leg] - datas[leg].qd(1));
    // legCommand->tau_knee_ff[leg] = legCommand->kp_knee[leg] * (legCommand->q_des_knee[leg] - datas[leg].q(2)) + 
    //                               legCommand->kd_knee[leg] * (legCommand->qd_des_knee[leg] - datas[leg].qd(2));
    
    // if(flags < 10){
    //   std::cout << "commands Q value is[ : " << leg << "] " << legCommand->tau_abad_ff[leg] << ", " << legCommand->tau_hip_ff[leg] << ", " << legCommand->tau_knee_ff[leg] << std::endl;
    //   std::cout << "commands P value is :" << jointAngle << std::endl;
    //   std::cout << "commands A value is :" << datas[leg].q << std::endl;
    // }
    
    legCommand->flags[leg] = _legsEnabled ? 1 : 0;
  }
}

  flags = flags + 1;
}

template struct LegControllerCommand<double>;
template struct LegControllerCommand<float>;

template struct LegControllerData<double>;
template struct LegControllerData<float>;

template class LegController<double>;
template class LegController<float>;

/*!
 * Compute the position of the foot and its Jacobian.  This is done in the local
 * leg coordinate system. If J/p are NULL, the calculation will be skipped.
 */
template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                   Vec3<T>* p, int leg) {
  T l1 = quad._abadLinkLength;
  T l2 = quad._hipLinkLength;
  T l3 = quad._kneeLinkLength;
  T l4 = quad._kneeLinkY_offset;
  T sideSign = quad.getSideSign(leg);

  T s1 = std::sin(q(0));
  T s2 = std::sin(q(1));
  T s3 = std::sin(q(2));

  T c1 = std::cos(q(0));
  T c2 = std::cos(q(1));
  T c3 = std::cos(q(2));

  T c23 = c2 * c3 - s2 * s3;
  T s23 = s2 * c3 + c2 * s3;

  if (J) {
    J->operator()(0, 0) = 0;
    J->operator()(0, 1) = l3 * c23 + l2 * c2;
    J->operator()(0, 2) = l3 * c23;
    J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1;
    J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J->operator()(1, 2) = -l3 * s1 * s23;
    J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1;
    J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    J->operator()(2, 2) = l3 * c1 * s23;
  }

  if (p) {
    p->operator()(0) = l3 * s23 + l2 * s2;
    p->operator()(1) = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p->operator()(2) = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
}

template void computeLegJacobianAndPosition<double>(Quadruped<double>& quad,
                                                    Vec3<double>& q,
                                                    Mat3<double>* J,
                                                    Vec3<double>* p, int leg);
template void computeLegJacobianAndPosition<float>(Quadruped<float>& quad,
                                                   Vec3<float>& q,
                                                   Mat3<float>* J,
                                                   Vec3<float>* p, int leg);


template <typename T>
void computeLegIK(Quadruped<T>& quad, Vec3<T>& pDes, Vec3<T>* qDes, int leg) {
  T l1 = quad._abadLinkLength + quad._kneeLinkY_offset;
  T l2 = quad._hipLinkLength;
  T l3 = quad._kneeLinkLength;
  T sideSign = quad.getSideSign(leg);

  T temp1 = (pDes(0) * pDes(0) + pDes(1) * pDes(1) + pDes(2) * pDes(2) - l1 * l1 - l2 * l2 - l3 * l3) / (2 * l2 * l3);
  qDes->operator()(2) = acos(temp1);

  T k1 = temp1;
  T k2 = sqrt(1 - k1 * k1);
  T temp2 = (l3 * k1 + l2) * (l3 * k1 + l2) + l3 * l3 * k2 * k2;
  T temp3 = 2 * pDes(0) * (l3 * k1 + l2);
  T temp4 = pDes(0) * pDes(0) - l3 * l3 * k2 * k2;
  T temp5 = (temp3 - sqrt(temp3 * temp3 - 4 * temp2 * temp4)) / (2 * temp2);
  qDes->operator()(1) = asin(temp5);

  T temp6 = sideSign * l1 + sqrt(pDes(1) * pDes(1) + pDes(2) * pDes(2) - l1 * l1);
  T temp7 = sideSign * l1 - sqrt(pDes(1) * pDes(1) + pDes(2) * pDes(2) - l1 * l1);
  T temp8 = temp6 * temp6 + temp7 * temp7;
  T temp9 = 2 * temp6 * (pDes(1) + pDes(2));
  T temp10 = (pDes(1) + pDes(2)) * (pDes(1) + pDes(2)) - temp7 * temp7;
  T temp11 = (temp9 + sqrt(temp9 * temp9 - 4 * temp8 * temp10)) / (2 * temp8);
  qDes->operator()(0) = asin(temp11);
}

// template <typename T>
// void computeLegIK(Quadruped<T>& quad, Vec3<T> pDes, std::vector<double> &qDes, int leg) {
//   T l1 = quad._abadLinkLength + quad._kneeLinkY_offset;
//   T l2 = quad._hipLinkLength;
//   T l3 = quad._kneeLinkLength;
//   T sideSign = quad.getSideSign(leg);

//   T tempL = sqrt(pDes[0] * pDes[0] + pDes[1] * pDes[1] + pDes[2] * pDes[2]);
//   T tempL23 = sqrt(tempL * tempL - l1 * l1);
//   T angle1 = fabs(
//       acos((l1 * l1 + tempL * tempL - tempL23 * tempL23) / (2 * l1 * tempL)));
//   T tempAngle1 = acos(fabs(pDes[1]) / tempL);
//   qDes[0] = sideSign * (angle1 - tempAngle1);
//   T angle2 = fabs(acos((l2 * l2 + tempL * tempL - l3 * l3) / (2 * l2 * tempL)));
//   T tempAngle2 = asin(pDes[0] / tempL);
//   qDes[1] = -(angle2 - tempAngle2);
//   T tempAngle3 =
//       fabs(acos((l2 * l2 + l3 * l3 - tempL * tempL) / (2 * l2 * l3)));
//   qDes[2] = 3.1415926535898 - tempAngle3;
// }
