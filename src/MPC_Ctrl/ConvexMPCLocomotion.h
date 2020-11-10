#ifndef _CONVEXMPCLOCOMOTION_H
#define _CONVEXMPCLOCOMOTION_H

#include "Controllers/FootSwingTrajectory.h"
#include "Controllers/ControlFSMData.h"
#include "SparseCMPC.h"
#include "Utilities/cppTypes.h"
#include "Gait.h"
#include <fstream> 
#include <sys/time.h>

#include <cstdio>
#include <vector>

using Eigen::Array4f;
using Eigen::Array4i;


template<typename T>
struct CMPC_Result {
  LegControllerCommand<T> commands[4];
  Vec4<T> contactPhase;
};

struct CMPC_Jump {
  static constexpr int START_SEG = 6;
  static constexpr int END_SEG = 0;
  static constexpr int END_COUNT = 2;
  bool jump_pending = false;
  bool jump_in_progress = false;
  bool pressed = false;
  int seen_end_count = 0;
  int last_seg_seen = 0;
  int jump_wait_counter = 0;

  void debug(int seg) {
    (void)seg;
    //printf("[%d] pending %d running %d\n", seg, jump_pending, jump_in_progress);
  }

  void trigger_pressed(int seg, bool trigger) {
    (void)seg;
    if(!pressed && trigger) {
      if(!jump_pending && !jump_in_progress) {
        jump_pending = true;
        //printf("jump pending @ %d\n", seg);
      }
    }
    pressed = trigger;
  }

  bool should_jump(int seg) {
    debug(seg);

    if(jump_pending && seg == START_SEG) {
      jump_pending = false;
      jump_in_progress = true;
      //printf("jump begin @ %d\n", seg);
      seen_end_count = 0;
      last_seg_seen = seg;
      return true;
    }

    if(jump_in_progress) {
      if(seg == END_SEG && seg != last_seg_seen) {
        seen_end_count++;
        if(seen_end_count == END_COUNT) {
          seen_end_count = 0;
          jump_in_progress = false;
          //printf("jump end @ %d\n", seg);
          last_seg_seen = seg;
          return false;
        }
      }
      last_seg_seen = seg;
      return true;
    }

    last_seg_seen = seg;
    return false;
  }
};


class ConvexMPCLocomotion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConvexMPCLocomotion(float _dt, int _iterations_between_mpc);
  void initialize();

  template<typename T>
  void run(Quadruped<T> &_quadruped, LegController<T> &_legController, StateEstimatorContainer<float> &_stateEstimator,
          DesiredStateCommand<T> &_desiredStateCommand, std::vector<double> gamepadCommand, int gaitType = 9, int robotMode = 0);
  // void _SetupCommand(StateEstimatorContainer<float> &_stateEstimator, std::vector<double> gamepadCommand);
  bool currently_jumping = false;

  Vec3<float> pBody_des;
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[4];
  Vec3<float> vFoot_des[4];
  Vec3<float> aFoot_des[4];

  Vec3<float> Fr_des[4];

  Vec4<float> contact_state;

private:
  void _SetupCommand(StateEstimatorContainer<float> &_stateEstimator, std::vector<double> gamepadCommand);

  float _yaw_turn_rate = 0.;
  float _yaw_des;

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;

  // High speed running
  //float _body_height = 0.34;
  float _body_height = 0.29;

  float _body_height_running = 0.29;
  float _body_height_jumping = 0.36;

  void recompute_timing(int iterations_per_mpc);
  void updateMPCIfNeeded(int* mpcTable, StateEstimatorContainer<float> &_stateEstimator, bool omniMode);
  void solveDenseMPC(int *mpcTable, StateEstimatorContainer<float> &_stateEstimator);
  void solveSparseMPC(int *mpcTable, StateEstimatorContainer<float> &_stateEstimator);
  void initSparseMPC();
  int iterationsBetweenMPC;  //15
  int horizonLength;    //10
  int default_iterations_between_mpc;
  float dt;  //0.002
  float dtMPC; //0.03
  int iterationCounter = 0;  //
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  OffsetDurationGait aio, trotting, bounding, pronking, jumping, galloping, standing, trotRunning, walking, walking2, pacing;
  // MixedFrequncyGait random, random2;
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance, Kp1;
  bool firstRun = true;
  bool firstSwing[4];  //true
  float swingTimeRemaining[4];
  float stand_traj[6];
  int current_gait;
  int gaitNumber;

  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  float x_comp_integral = 0;
  Vec3<float> pFoot[4];
  CMPC_Result<float> result;
  float trajAll[12*36];
  float myflags = 0;

  CMPC_Jump jump_state;

  vectorAligned<Vec12<double>> _sparseTrajectory;

  SparseCMPC _sparseCMPC;

};


#endif //CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
