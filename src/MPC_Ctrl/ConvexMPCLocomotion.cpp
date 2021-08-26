#include "ConvexMPCLocomotion.h"

#include <iostream>

#include "Utilities/Timer.h"
#include "Utilities/Utilities_print.h"
#include "convexMPC_interface.h"
// #include "../../../../common/FootstepPlanner/GraphSearch.h"

// #include "Gait.h"

//#define DRAW_DEBUG_SWINGS
//#define DRAW_DEBUG_PATH

////////////////////
// Controller
// 参考Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive
// Control 一个步态周期由horizonLength(10)个mpc周期组成 步态按1KHz处理
// mpc计数间隔为30左右 一毫秒计数一次来控制频率 即一个mpc周期为30ms 则步态周期为
// 10*30 =300ms
////////////////////

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc)
    : iterationsBetweenMPC(_iterations_between_mpc),  //控制频率用  15
      horizonLength(14),
      dt(_dt),  // 0.002
      trotting(horizonLength, Vec4<int>(0, horizonLength/2.0, horizonLength/2.0, 0),
      Vec4<int>(horizonLength/2.0, horizonLength/2.0, horizonLength/2.0, horizonLength/2.0), "Trotting"),
      bounding(horizonLength, Vec4<int>(7, 7, 0, 0), Vec4<int>(6, 6, 6, 6), "Bounding"),
      // bounding(horizonLength,
      // Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
      pronking(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(6, 6, 6, 6), "Pronking"),
      jumping(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(3, 3, 3, 3), "Jumping"),
      galloping(horizonLength, Vec4<int>(0, 4, 7, 11), Vec4<int>(7, 7, 7, 7), "Galloping"),
      standing( horizonLength, Vec4<int>(0, 0, 0, 0),  Vec4<int>(14, 14, 14, 14), "Standing"),
      trotRunning(horizonLength, Vec4<int>(0, 7, 7, 0), Vec4<int>(6, 6, 6, 6), "Trot Running"),
      walking(horizonLength, Vec4<int>(0, horizonLength/2.0, horizonLength/4.0, 3.0*horizonLength/4.0),
      Vec4<int>(3.0*horizonLength/4.0,3.0*horizonLength/4.0,3.0*horizonLength/4.0,3.0*horizonLength/4.0), "Walking"),
      walking2(horizonLength, Vec4<int>(0, 7, 7, 0), Vec4<int>(10, 10, 10, 10), "Walking2"),
      pacing(horizonLength, Vec4<int>(7, 0, 7, 0), Vec4<int>(7, 7, 7, 7), "Pacing"),
      aio(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(14, 14, 14, 14), "aio") {
  dtMPC = dt * iterationsBetweenMPC;  // 0.03
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt,
         iterationsBetweenMPC, dtMPC);  // 0.002, 15, 0.03
  // setup_problem(dtMPC, horizonLength, 0.4, 120);
  // setup_problem(dtMPC, horizonLength, 0.4, 650); // DH
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for (int i = 0; i < 4; i++) firstSwing[i] = true;

  initSparseMPC();

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();
  for (int i = 0; i < 4; i++) f_ff[i].setZero();
}

void ConvexMPCLocomotion::initialize() {
  for (int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
}

void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc) {
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

//设置期望值
void ConvexMPCLocomotion::_SetupCommand(
    StateEstimatorContainer<float>& _stateEstimator,
    std::vector<double> gamepadCommand) {
  _body_height = 0.25;

  float x_vel_cmd, y_vel_cmd, yaw_vel_cmd;
  float x_filter(0.01), y_filter(0.006), yaw_filter(0.03);

  //手柄数据先暂时设置为0，后面再给手柄赋值   旋转角速度和x,y方向上的线速度
  x_vel_cmd = gamepadCommand[0];
  y_vel_cmd = gamepadCommand[1];
  yaw_vel_cmd = gamepadCommand[2];

  _x_vel_des = _x_vel_des * (1 - x_filter) + x_vel_cmd * x_filter;  //一阶低通数字滤波
  _y_vel_des = _y_vel_des * (1 - y_filter) + y_vel_cmd * y_filter;
  _yaw_turn_rate = _yaw_turn_rate * (1 - yaw_filter) + yaw_vel_cmd * yaw_filter;
  if(_x_vel_des > 2.0) {
    _x_vel_des = 2.0;
  } else if(_x_vel_des < -1.0) {
    _x_vel_des = -1.0;
  }
  if(_y_vel_des > 0.6) {
    _y_vel_des = 0.6;
  } else if(_y_vel_des < -0.6) {
    _y_vel_des = -0.6;
  }
  _yaw_des = _stateEstimator.getResult().rpy[2] +
             dt * _yaw_turn_rate;  //涉及到了状态估计中的欧拉角

  //确保机器人不会因为摩擦力的原因在yaw方向产生旋转误差
  if((abs(_stateEstimator.getResult().rpy[2] - _yaw_des_true) > 5.0)){
    // _yaw_des_true = 3.14 * _stateEstimator.getResult().rpy[2] / abs(_stateEstimator.getResult().rpy[2]);
    _yaw_des_true = _stateEstimator.getResult().rpy[2];
  }
  _yaw_des_true =_yaw_des_true + dt * _yaw_turn_rate;

  _roll_des = 0.;
  _pitch_des = 0.;
}

template <>
void ConvexMPCLocomotion::run(Quadruped<float>& _quadruped,
                              LegController<float>& _legController,
                              StateEstimatorContainer<float>& _stateEstimator,
                              DesiredStateCommand<float>& /*_desiredStateCommand*/,
                              std::vector<double> gamepadCommand,
                              int gaitType, int robotMode) {
  bool omniMode = false;
  // Command Setup
  _SetupCommand(_stateEstimator, gamepadCommand);

  gaitNumber = gaitType;  // data.userParameters->cmpc_gait; 步态默认为trot

  if (gaitNumber >= 20) {
    gaitNumber -= 20;
    omniMode = true;
  }

  auto& seResult = _stateEstimator.getResult();  //状态估计器

  // Check if transition to standing 检查是否过渡到站立
  if (((gaitNumber == 4) && current_gait != 4) || firstRun) {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = 0.21;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  Gait* gait = &trotting;
  if(robotMode == 0) {
    if (gaitNumber == 1)
      gait = &bounding;
    else if (gaitNumber == 2)
      gait = &pronking;
    // else if(gaitNumber == 3)
    //   gait = &random;
    else if (gaitNumber == 4)
      gait = &standing;
    else if (gaitNumber == 5)
      gait = &trotRunning;
    // else if(gaitNumber == 6)
    //   gait = &random2;
    else if (gaitNumber == 7)
      gait = &galloping;
    else if (gaitNumber == 8)
      gait = &pacing;
    else if (gaitNumber == 9)
      gait = &trotting;
    else if (gaitNumber == 10)
      gait = &walking;
    else if (gaitNumber == 11)
      gait = &walking2;
  } else if(robotMode == 1) {
    int h = 10;
    double vBody = sqrt(_x_vel_des*_x_vel_des)+(_y_vel_des*_y_vel_des);
    gait = &aio;
    gaitNumber = 9;  // Trotting
    if(gait->getCurrentGaitPhase() == 0) {
      if(vBody < 0.002) {
        if(abs(_yaw_turn_rate) < 0.01) {
          gaitNumber = 4;  // Standing
          if(gait->getGaitHorizon() != h) {
            iterationCounter = 0;
          }
          gait->setGaitParam(h, Vec4<int>(0, 0, 0, 0), Vec4<int>(h, h, h, h), "Standing");
        } else {
          h = 10;
          if(gait->getGaitHorizon() != h) {
            iterationCounter = 0;
          }
          gait->setGaitParam(h, Vec4<int>(0, h / 2, h / 2, 0),
                                Vec4<int>(h / 2, h / 2, h / 2, h / 2), "trotting");
        }
      } else {
        if(vBody <= 0.2) {
          h = 16;
          if(gait->getGaitHorizon() != h) {
            iterationCounter = 0;
          }
          gait->setGaitParam(h,
                  Vec4<int>(0, 1 * h / 2, 1 * h / 4, 3 * h / 4),
                  Vec4<int>(3 * h / 4, 3 * h / 4, 3 * h / 4, 3 * h / 4), "Walking");
        } else if(vBody > 0.2 && vBody <= 0.4) {
          h = 16;
          if(gait->getGaitHorizon() != h) {
            iterationCounter = 0;
          }
          gait->setGaitParam(h,
                  Vec4<int>(0, 1 * h / 2, h*((5.0/4.0)*vBody), h*((5.0/4.0)*vBody+(1.0/2.0))),
                  Vec4<int>(h*((-5.0/4.0)*vBody+1.0), h*((-5.0/4.0)*vBody+1.0),
                            h*((-5.0/4.0)*vBody+1.0), h*((-5.0/4.0)*vBody+1.0)), "Walking2trotting");
        } else if(vBody > 0.4 && vBody <= 1.4) {
          h = 14;
          if(gait->getGaitHorizon() != h) {
            iterationCounter = 0;
          }
          gait->setGaitParam(h, Vec4<int>(0, h / 2, h / 2, 0),
                                Vec4<int>(h / 2, h / 2, h / 2, h / 2), "trotting");
        } else {
          // h = 10;
          h = -20.0*vBody+42.0;
          if(h < 10) h = 10;
          if(gait->getGaitHorizon() != h) {
            iterationCounter = 0;
          }
          gait->setGaitParam(h, Vec4<int>(0, h / 2, h / 2, 0),
                                Vec4<int>(h / 2, h / 2, h / 2, h / 2), "trotting");

          // std::cout << vBody << " " << h << " " << h / 2 << std::endl;
        }
      }
    }
    horizonLength = h;
  } else {
    std::cout << "err robot mode!!!" << std::endl;
  }

  current_gait = gaitNumber;
  gait->setIterations(iterationsBetweenMPC, iterationCounter);  //步态周期计算

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,
                          0);  //身体坐标系下的期望线速度
  Vec3<float> v_des_world = omniMode
                                ? v_des_robot
                                : seResult.rBody.transpose() *
                                      v_des_robot;  //世界坐标系下的期望线速度
  Vec3<float> v_robot = seResult.vWorld;  //世界坐标系下的机器人实际速度

  // Integral-esque pitche and roll compensation
  // 积分达到补偿值*******************************
  // 为了保持在运动过程中身体与地面平行
  if (fabs(v_robot[0]) > .2)  // avoid dividing by zero
  {
    rpy_int[1] += dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
  }
  if (fabs(v_robot[1]) > 0.1) {
    rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];
  }

  //初始角度限幅
  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);  //-0.25~0.25
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];  // compensation 补偿值
  rpy_comp[0] = v_robot[1] * rpy_int[0];  // turn off for pronking

  //得到世界坐标系下的足端位置
  //机身坐标+机身旋转矩阵^T*（侧摆关节在机身下坐标+足底在侧摆关节下坐标）
  for (int i = 0; i < 4; i++) {
    pFoot[i] = seResult.position +
               seResult.rBody.transpose() *
                   (_quadruped.getHipLocation(i) + _legController.datas[i].p);
    // pFoot[i] = _legController.datas[i].p;
  }

  Vec3<float> error;
  if (gait != &standing) {  //非站立下的期望位置，通过累加期望速度完成
    world_position_desired +=
        dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if (firstRun) {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];

    for (int i = 0; i < 4; i++)  //足底摆动轨迹
    {
      footSwingTrajectories[i].setHeight(0.06);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);  // set p0
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);    // set pf
    }
    firstRun = false;
  }

  // foot placement
  for (int l = 0; l < 4; l++) {
    swingTimes[l] = gait->getCurrentSwingTime(
        dtMPC, l);  // return dtMPC * _stance  0.026 * 5 = 0.13
                    // dtMPC的值变为了0.026，外部给修改的赋值
  }

  float side_sign[4] = {-1, 1, -1, 1};
  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  // float interleave_gain = -0.13;
  float interleave_gain = -0.2;
  // float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);
  for (int i = 0; i < 4; i++) {
    if (firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }

    footSwingTrajectories[i].setHeight(0.06);
    Vec3<float> offset(0, side_sign[i] * .065, 0);

    Vec3<float> pRobotFrame = (_quadruped.getHipLocation(i) +
                               offset);  //得到身体坐标系下的hip关节坐标

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time =
        gait->getCurrentStanceTime(dtMPC, i);  // stance_time = 0.13

    Vec3<float> pYawCorrected =
        coordinateRotation(CoordinateAxis::Z,
                           -_yaw_turn_rate * stance_time / 2) *
        pRobotFrame;  //机身旋转yaw后，得到在机身坐标系下的hip坐标

    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    //世界坐标系下hip坐标 以剩余摆动时间内匀速运动来估计
    Vec3<float> Pf = seResult.position +
                     seResult.rBody.transpose() *
                         (pYawCorrected + des_vel * swingTimeRemaining[i]);
    // float p_rel_max = 0.35f;
    float p_rel_max = 0.3f;

    // Using the estimated velocity is correct
    // Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
    float pfx_rel = seResult.vWorld[0] * (.5 + 0.0) *
                        stance_time +  //_parameters->cmpc_bonus_swing = 0.0
                    .03f * (seResult.vWorld[0] - v_des_world[0]) +
                    (0.5f * sqrt(seResult.position[2] / 9.81f)) *
                        (seResult.vWorld[1] * _yaw_turn_rate);

    float pfy_rel = seResult.vWorld[1] * .5 * stance_time * 1.0 + //dtMPC +
                    .03f * (seResult.vWorld[1] - v_des_world[1]) +
                    (0.5f * sqrt(seResult.position[2] / 9.81f)) *
                        (-seResult.vWorld[0] * _yaw_turn_rate);
    if (i == 1) {
      // std::cout << "pf0 = " << (seResult.rBody*(Pf- seResult.position)).transpose() << " " << std::endl;
      // std::cout << pfx_rel << " " << pfy_rel << std::endl;
      // std::cout << 0.5f * sqrt(seResult.position[2] / 9.81f) * (seResult.vWorld[1] * _yaw_turn_rate) << " "
      //           << (0.5f * sqrt(seResult.position[2] / 9.81f)) * (-seResult.vWorld[0] * _yaw_turn_rate) << std::endl;
      // std::cout << (0.5f * seResult.position[2] / 9.81f) * (seResult.vWorld[0] * _yaw_turn_rate) << std::endl;
      // std::cout << _yaw_turn_rate << std::endl;
    }
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel;
    // Pf[2] = -0.003;
    Pf[2] = 0.0;
    footSwingTrajectories[i].setFinalPosition(Pf);  //最终得到足底的位置，并作为轨迹终点 世界坐标系下的落足点
  }
  // std::cout << std::endl;

  // calc gait
  iterationCounter++;

  // load LCM leg swing gains
  Kp << 700, 0, 0, 0, 700, 0, 0, 0, 200;
  Kp_stance = 0.0 * Kp;

  Kd << 10, 0, 0, 0, 10, 0, 0, 0, 10;
  Kd_stance = 1.0 * Kd;
  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();
  updateMPCIfNeeded(mpcTable, _stateEstimator, omniMode);

  //  StateEstimator* se = hw_i->state_estimator;
  Vec4<float> se_contactState(0, 0, 0, 0);

  bool use_wbc = false;

  for (int foot = 0; foot < 4; foot++) {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    if (swingState > 0)  // foot is in swing
    {
      if (firstSwing[foot]) {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(
          swingState, swingTimes[foot]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();

      Vec3<float> pDesLeg =
          seResult.rBody *
              (pDesFootWorld -
               seResult.position)  //侧摆关节坐标系下的足端坐标
                                   //(此处先改为身体坐标系下的足端坐标)
          - _quadruped.getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

      if (!use_wbc) {
        // Update leg control command regardless of the usage of WBIC
        _legController.commands[foot].pDes = pDesLeg;
        _legController.commands[foot].vDes = vDesLeg;
        if (foot == 1 || foot == 3) {
          _legController.commands[foot].kpCartesian = Kp;
          _legController.commands[foot].kdCartesian = Kd;
        } else {
          _legController.commands[foot].kpCartesian = 1*Kp;
          _legController.commands[foot].kdCartesian = 1*Kd;
        }
      }
    } else  // foot is in stance
    {
      firstSwing[foot] = true;

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg =
          seResult.rBody * (pDesFootWorld - seResult.position) -
          _quadruped.getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      if (!use_wbc) {
        _legController.commands[foot].pDes = pDesLeg;
        _legController.commands[foot].vDes = vDesLeg;

        if (foot == 1 || foot == 3) {
          _legController.commands[foot].kdCartesian = Kd_stance;
        } else {
          _legController.commands[foot].kdCartesian = 1*Kd_stance;
        }

        _legController.commands[foot].forceFeedForward = f_ff[foot];
        _legController.commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;

      } else {  // Stance foot damping
        _legController.commands[foot].pDes = pDesLeg;
        _legController.commands[foot].vDes = vDesLeg;
        _legController.commands[foot].kpCartesian = 0. * Kp_stance;
        _legController.commands[foot].kdCartesian = Kd_stance;
      }
      se_contactState[foot] = contactState;

      // Update for WBC
      // Fr_des[foot] = -f_ff[foot];
    }
  }
  // se->set_contact_state(se_contactState); todo removed
  _stateEstimator.setContactPhase(se_contactState);

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = 0.;
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  // contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // END of WBC Update
}

void ConvexMPCLocomotion::updateMPCIfNeeded(
    int* mpcTable, StateEstimatorContainer<float>& _stateEstimator,
    bool omniMode) {
  // iterationsBetweenMPC = 30;
  if ((iterationCounter % iterationsBetweenMPC) == 0) {
    auto seResult = _stateEstimator.getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
    Vec3<float> v_des_world =
        omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    // float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

    // printf("Position error: %.3f, integral %.3f\n", pxy_err[0],
    // x_comp_integral);

    if (current_gait == 4) {
      float trajInitial[12] = {
          _roll_des,
          _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
          (float)stand_traj[5] /*+(float)stateCommand->data.stateDes[11]*/,
          (float)stand_traj[0] /*+(float)fsm->main_control_settings.p_des[0]*/,
          (float)stand_traj[1] /*+(float)fsm->main_control_settings.p_des[1]*/,
          (float)_body_height /*fsm->main_control_settings.p_des[2]*/,
          0,
          0,
          0,
          0,
          0,
          0};

      for (int i = 0; i < horizonLength; i++)
        for (int j = 0; j < 12; j++) trajAll[12 * i + j] = trajInitial[j];
    }

    else {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      if (xStart - p[0] > max_pos_error) xStart = p[0] + 0.1;
      if (p[0] - xStart > max_pos_error) xStart = p[0] - 0.1;

      if (yStart - p[1] > max_pos_error) yStart = p[1] + 0.1;
      if (p[1] - yStart > max_pos_error) yStart = p[1] - 0.1;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      float trajInitial[12] = {(float)rpy_comp[0],  // 0
                               (float)rpy_comp[1],  // 1
                               _yaw_des_true,            // 2
                               // yawStart,    // 2
                               xStart,               // 3
                               yStart,               // 4
                               (float)_body_height,  // 5
                               0,                    // 6
                               0,                    // 7
                               _yaw_turn_rate,       // 8
                               v_des_world[0],       // 9
                               v_des_world[1],       // 10
                               0};                   // 11

      for (int i = 0; i < horizonLength; i++) {
        for (int j = 0; j < 12; j++) trajAll[12 * i + j] = trajInitial[j];

        if (i == 0)  // start at current position  TODO consider not doing this
        {
          // trajAll[2] = seResult.rpy[2];
          trajAll[2] = _yaw_des_true;
        } else {
          trajAll[12 * i + 3] =
              trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12 * i + 4] =
              trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12 * i + 2] =
              trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }

    Timer solveTimer;

    int cmpc_use_sparse = 0.0;

    if (cmpc_use_sparse > 0.5) {
      solveSparseMPC(mpcTable, _stateEstimator);
    } else {
      solveDenseMPC(mpcTable, _stateEstimator);
    }
    // printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }
}

void ConvexMPCLocomotion::solveDenseMPC(
    int* mpcTable, StateEstimatorContainer<float>& _stateEstimator) {
  auto seResult = _stateEstimator.getResult();

  // float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};

  float Q[12] = {2.5, 2.5, 10, 50, 50, 100, 0, 0, 0.5, 0.2, 0.2, 0.1};
  // float Q[12] = {0, 0, 0, 10, 10, 10, 0, 0, 0, 0, 0, 0};

  // float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5;  // make setting eventually
  // float alpha = 4e-7; // make setting eventually: DH
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  float r[12];
  for (int i = 0; i < 12; i++)
    r[i] = pFoot[i % 4][i / 4] - seResult.position[i / 4];

  // printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

  if (alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  // Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  setup_problem(dtMPC, horizonLength, 0.4, 120);
  // setup_problem(dtMPC,horizonLength,0.4,650); //DH
  update_x_drag(x_comp_integral);

  float cmpc_x_drag = 3.0;

  if (vxy[0] > 0.3 || vxy[0] < -0.3) {
    // x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC /
    // vxy[0];
    x_comp_integral += cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  // printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  int jcqp_max_iter = 10000;
  double jcqp_rho = 0.0000001;
  double jcqp_sigma = 0.00000001;
  double jcqp_alpha = 1.5;
  double jcqp_terminate = 0.1;
  double use_jcqp = 0.0;

  update_solver_settings(jcqp_max_iter, jcqp_rho, jcqp_sigma, jcqp_alpha,
                         jcqp_terminate, use_jcqp);
  // t1.stopPrint("Setup MPC");

  Timer t2;
  // cout << "dtMPC: " << dtMPC << "\n";
  // std::cout << "pFoot = " << std::endl;
  // for(int i=0; i<4; i++) {
  //   for(int j=0; j<3; j++) {
  //     std::cout << pFoot[i][j] << " ";
  //   }
  // }
  // std::cout << std::endl;
  update_problem_data_floats(p, v, q, w, r, yaw, weights, trajAll, alpha,
                             mpcTable);

  // std::cout << "the value is " << mpcTable << std::endl;

  // t2.stopPrint("Run MPC");
  // printf("MPC Solve time %f ms\n", t2.getMs());

  for (int leg = 0; leg < 4; leg++) {
    Vec3<float> f;
    for (int axis = 0; axis < 3; axis++) f[axis] = get_solution(leg * 3 + axis);

    // if(myflags < 50){
    // printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);
    // }

    f_ff[leg] = -seResult.rBody * f;
    // std::cout << "Foot " << leg << " force: " << f.transpose() << "\n";
    // std::cout << "Foot " << leg << " force: " << f_ff[leg].transpose() <<
    // "\n"; Update for WBC
    Fr_des[leg] = f;
  }
  myflags = myflags + 1;
}

void ConvexMPCLocomotion::solveSparseMPC(
    int* mpcTable, StateEstimatorContainer<float>& _stateEstimator) {
  // X0, contact trajectory, state trajectory, feet, get result!
  (void)mpcTable;
  auto seResult = _stateEstimator.getResult();

  std::vector<ContactState> contactStates;
  for (int i = 0; i < horizonLength; i++) {
    contactStates.emplace_back(mpcTable[i * 4 + 0], mpcTable[i * 4 + 1],
                               mpcTable[i * 4 + 2], mpcTable[i * 4 + 3]);
  }

  for (int i = 0; i < horizonLength; i++) {
    for (u32 j = 0; j < 12; j++) {
      _sparseTrajectory[i][j] = trajAll[i * 12 + j];
    }
  }

  Vec12<float> feet;
  for (u32 foot = 0; foot < 4; foot++) {
    for (u32 axis = 0; axis < 3; axis++) {
      feet[foot * 3 + axis] = pFoot[foot][axis] - seResult.position[axis];
    }
  }

  _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation,
                    seResult.omegaWorld);
  _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
  _sparseCMPC.setStateTrajectory(_sparseTrajectory);
  _sparseCMPC.setFeet(feet);
  _sparseCMPC.run();

  Vec12<float> resultForce = _sparseCMPC.getResult();

  for (u32 foot = 0; foot < 4; foot++) {
    Vec3<float> force(resultForce[foot * 3], resultForce[foot * 3 + 1],
                      resultForce[foot * 3 + 2]);
    printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
    f_ff[foot] = -seResult.rBody * force;
    Fr_des[foot] = force;
  }
}

void ConvexMPCLocomotion::initSparseMPC() {
  Mat3<double> baseInertia;
  baseInertia << 0.07, 0, 0, 0, 0.26, 0, 0, 0, 0.242;
  double mass = 9;
  double maxForce = 120;

  std::vector<double> dtTraj;
  for (int i = 0; i < horizonLength; i++) {
    dtTraj.push_back(dtMPC);
  }

  Vec12<double> weights;
  weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
  // weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

  _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
  _sparseCMPC.setFriction(1.0);
  _sparseCMPC.setWeights(weights, 4e-5);
  _sparseCMPC.setDtTrajectory(dtTraj);

  _sparseTrajectory.resize(horizonLength);
}