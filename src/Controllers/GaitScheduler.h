/*!
 * @file GaitScheduler.h
 * @brief Logic for fixed-gait timing
 */

#ifndef GAIT_SCHEDULER_H
#define GAIT_SCHEDULER_H

#include <iostream>

#include "Utilities/cppTypes.h"
#include "MIT_UserParameters.h"

/**
 * Enumerated gait types. Preplanned gaits are defined.
 * 枚举类型
 */
enum class GaitType
{
  STAND,
  STAND_CYCLE,
  STATIC_WALK,
  AMBLE,
  TROT_WALK,
  TROT,
  TROT_RUN,
  PACE,
  BOUND,
  ROTARY_GALLOP,
  TRAVERSE_GALLOP,
  PRONK,
  THREE_FOOT,
  CUSTOM,
  TRANSITION_TO_STAND
};

/**
 * Timing data for a gait 步态时间信息
 */
template <typename T>
struct GaitData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GaitData() { zero(); }

  // Zero out all of the data 把所有的数据归零
  void zero();

  // The current GaitType 当前GaitType
  GaitType _currentGait;

  // Next GaitType to transition into 下一步GaitType过渡到
  GaitType _nextGait;

  // Gait name string 步态的名字字符串
  std::string gaitName;

  // Gait descriptors 步态描述符
  T periodTimeNominal;     // overall period time to scale 总周期时间以规模
  T initialPhase;          // initial phase to offset 初始偏移相位
  T switchingPhaseNominal; // nominal phase to switch contacts 标称相位切换点
  int overrideable;        // 允许步态参数被覆写

  // Enable flag for each foot 启用每只脚的标记
  Eigen::Vector4i gaitEnabled; // enable gait controlled legs 启动步态控制腿

  // Time based descriptors 基于时间的描述符
  Vec4<T> periodTime;          // overall foot scaled gait period time 整个步态时间
  Vec4<T> timeStance;          // total stance time 总站立时间
  Vec4<T> timeSwing;           // total swing time 总摆动时间
  Vec4<T> timeStanceRemaining; // stance time remaining 剩余站立时间
  Vec4<T> timeSwingRemaining;  // swing time remaining 剩余摆动时间

  // Phase based descriptors 阶段基于描述符
  Vec4<T> switchingPhase; // phase to switch to swing 相切换到swing
  Vec4<T> phaseVariable;  // overall gait phase for each foot 每只脚的整体步态阶段
  Vec4<T> phaseOffset;    // nominal gait phase offsets 步态相位偏移
  Vec4<T> phaseScale;     // phase scale relative to variable 相对于变量的相位刻度
  Vec4<T> phaseStance;    // stance subphase 支撑相中相位
  Vec4<T> phaseSwing;     // swing subphase 摆动相中相位

  // Scheduled contact states 预定的接触状态
  Eigen::Vector4i contactStateScheduled; // contact state of the foot 脚的接触状态
  Eigen::Vector4i contactStatePrev;      // previous contact state of the foot 脚的先前接触状态
  Eigen::Vector4i touchdownScheduled;    // scheduled touchdown event flag 预定的触地事件标志
  Eigen::Vector4i liftoffScheduled;      // scheduled lift event flag 预定离地事件标值
};

/**
 * Utility to process GaitData and schedule foot steps and swings.
 * 实用程序，以处理步态数据和计划的步伐和摆动。
 */
template <typename T>
class GaitScheduler
{
public:
  // Constructors for the GaitScheduler
  GaitScheduler(MIT_UserParameters *_userParameters, float _dt); //构造函数
  ~GaitScheduler(){};

  // Initialize the Gait Scheduler//初始化
  void initialize();

  // Iteration step for scheduler logic 逻辑循环
  void step();

  // Creates a new gait from predefined library
  //修改步态
  void modifyGait();
  //创建步态
  void createGait();
  //计算辅助步态数据
  void calcAuxiliaryGaitData();

  // Prints the characteristic info and curret state
  void printGaitInfo();

  // Struct containing all of the gait relevant data 储存所有步态相关数据
  GaitData<T> gaitData;

  // Natural gait modifiers  自然步态修饰符
  //周期时间
  T period_time_natural = 0.5;
  //摆动向接触切换点
  T switching_phase_natural = 0.5;
  //摆动时间
  T swing_time_natural = 0.25;

private:
  // The quadruped model
  // Quadruped<T>& _quadruped;
  MIT_UserParameters *userParameters;

  // Control loop timestep change
  T dt;

  // Phase change at each step
  T dphase;

  // Choose how often to print info, every N iterations
  int printNum = 5; // N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int printIter = 0;
};

#endif