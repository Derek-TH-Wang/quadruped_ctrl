/*!
 * @file GaitScheduler.cpp
 * @brief Logic for fixed-gait timing
 */

/*========================= Gait Scheduler ============================*/
/**
 *和ConvexMPC中的gait基本流程相同 
 */

#include "Controllers/GaitScheduler.h"

/*=========================== Gait Data ===============================*/

/**
 * Reset gait data to zeros
 */
template <typename T>
void GaitData<T>::zero()
{
  // Stop any gait transitions
  _nextGait = _currentGait;

  // General Gait descriptors
  periodTimeNominal = 0.0;     // overall period time to scale
  initialPhase = 0.0;          // initial phase to offset
  switchingPhaseNominal = 0.0; // nominal phase to switch contacts
  overrideable = 0;            // allows the gait parameters to be overridden

  // Enable flag for each foot
  gaitEnabled = Eigen::Vector4i::Zero(); // enable gait controlled legs

  // Time based descriptors
  periodTime = Vec4<T>::Zero();          // overall gait period time
  timeStance = Vec4<T>::Zero();          // total stance time
  timeSwing = Vec4<T>::Zero();           // total swing time
  timeStanceRemaining = Vec4<T>::Zero(); // stance time remaining
  timeSwingRemaining = Vec4<T>::Zero();  // swing time remaining

  // Phase based descriptors
  switchingPhase = Vec4<T>::Zero(); // phase to switch to swing
  phaseVariable = Vec4<T>::Zero();  // overall gait phase for each foot
  phaseOffset = Vec4<T>::Zero();    // nominal gait phase offsets
  phaseScale = Vec4<T>::Zero();     // phase scale relative to variable
  phaseStance = Vec4<T>::Zero();    // stance subphase
  phaseSwing = Vec4<T>::Zero();     // swing subphase

  // Scheduled contact states
  contactStateScheduled = Eigen::Vector4i::Zero(); // contact state of the foot
  contactStatePrev = Eigen::Vector4i::Zero();      // previous contact state of the foot
  touchdownScheduled = Eigen::Vector4i::Zero();    // scheduled touchdown flag
  liftoffScheduled = Eigen::Vector4i::Zero();      // scheduled liftoff flag
}

template struct GaitData<double>;
template struct GaitData<float>;

/*========================= Gait Scheduler ============================*/

/**
 * Constructor to automatically setup a basic gait
 */
template <typename T>
GaitScheduler<T>::GaitScheduler(MIT_UserParameters *_userParameters, float _dt)
{
  initialize();
  userParameters = _userParameters;
  dt = _dt;
}

/**
 * Initialize the gait data
 */
template <typename T>
void GaitScheduler<T>::initialize()
{
  std::cout << "[GAIT] Initialize Gait Scheduler" << std::endl;

  // Start the gait in a trot since we use this the most 开始步态在小跑，因为我们使用这个最多
  gaitData._currentGait = GaitType::STAND;

  // Zero all gait data
  gaitData.zero();

  // Create the gait from the nominal initial 创建步态从标称初始值
  createGait();
  period_time_natural = gaitData.periodTimeNominal;
  switching_phase_natural = gaitData.switchingPhaseNominal;
}

/**
 * Executes the Gait Schedule step to calculate values for the defining
 * gait parameters.
 */
template <typename T>
void GaitScheduler<T>::step()
{

  // Modify the gait with settings 修改步态设置（切换步态）
  modifyGait();

  //非站立
  if (gaitData._currentGait != GaitType::STAND)
  {
    // Track the reference phase variable 跟踪参考相位变量
    gaitData.initialPhase = fmod((gaitData.initialPhase + (dt / gaitData.periodTimeNominal)), 1);
  }

  // Iterate over the feet 遍历四足
  for (int foot = 0; foot < 4; foot++)
  {
    // Set the previous contact state for the next timestep 设置前一个接触状态
    gaitData.contactStatePrev(foot) = gaitData.contactStateScheduled(foot);

    if (gaitData.gaitEnabled(foot) == 1)
    {
      // Monotonic time based phase incrementation 单调时基相位增量
      if (gaitData._currentGait == GaitType::STAND)
      {
        // Don't increment the phase when in stand mode 在站立模式下，不要增加相位
        dphase = 0.0;
      }
      else
      {
        dphase = gaitData.phaseScale(foot) * (dt / gaitData.periodTimeNominal); //每次循环相位增量
      }

      // Find each foot's current phase 找到脚的当前相位 通过增加相位增量 并取余数来实现0~1
      gaitData.phaseVariable(foot) =
          fmod((gaitData.phaseVariable(foot) + dphase), 1);

      // Check the current contact state 检测当前接触状态
      if (gaitData.phaseVariable(foot) <= gaitData.switchingPhase(foot)) //当前为接触状态
      {
        // Foot is scheduled to be in contact  足部预定进行接触
        gaitData.contactStateScheduled(foot) = 1;

        // Stance subphase calculation  在支撑相中的相位
        gaitData.phaseStance(foot) =
            gaitData.phaseVariable(foot) / gaitData.switchingPhase(foot);

        // Swing phase has not started since foot is in stance  摇摆阶段还没有开始，因为脚在支撑
        gaitData.phaseSwing(foot) = 0.0;

        // Calculate the remaining time in stance 计算剩余支撑时间
        gaitData.timeStanceRemaining(foot) =
            gaitData.periodTime(foot) *
            (gaitData.switchingPhase(foot) - gaitData.phaseVariable(foot));

        // Foot is in stance, no swing time remaining 剩余摆动时间 当前腿在支撑 没有剩余摆动时间
        gaitData.timeSwingRemaining(foot) = 0.0;

        // First contact signifies scheduled touchdown  第一次接触表示预定着陆
        if (gaitData.contactStatePrev(foot) == 0)//之前为摆动
        {
          // Set the touchdown flag to 1 设置着陆标志为1
          gaitData.touchdownScheduled(foot) = 1;
        }
        else
        {
          // Set the touchdown flag to 0 设置着陆标志为0
          gaitData.touchdownScheduled(foot) = 0;
        }
      }
      else
      {
        // Foot is not scheduled to be in contact 足端没有预定进行接触
        gaitData.contactStateScheduled(foot) = 0;

        // Stance phase has completed since foot is in swing  支撑相结束 在支撑相中的相位
        gaitData.phaseStance(foot) = 1.0;

        // Swing subphase calculation  Swing相中的相位计算
        gaitData.phaseSwing(foot) =
            (gaitData.phaseVariable(foot) - gaitData.switchingPhase(foot)) /
            (1.0 - gaitData.switchingPhase(foot));

        // Foot is in swing, no stance time remaining 摆动没有支撑时间剩余
        gaitData.timeStanceRemaining(foot) = 0.0;

        // Calculate the remaining time in swing 计算剩余摆动时间
        gaitData.timeSwingRemaining(foot) =
            gaitData.periodTime(foot) * (1 - gaitData.phaseVariable(foot));

        // First contact signifies scheduled touchdown  第一次接触表示预定着陆
        if (gaitData.contactStatePrev(foot) == 1)//之前为接触
        {
          // Set the liftoff flag to 1 设置离地标志为1
          gaitData.liftoffScheduled(foot) = 1;
        }
        else
        {
          // Set the liftoff flag to 0 设置离地标志为1
          gaitData.liftoffScheduled(foot) = 0;
        }
      }
    }
    else
    {
      // Leg is not enabled 腿未启用
      gaitData.phaseVariable(foot) = 0.0;

      // Foot is not scheduled to be in contact 脚没有接触计划
      gaitData.contactStateScheduled(foot) = 0;
    }
  }
}

/**
 *步态切换
 */
template <typename T>
void GaitScheduler<T>::modifyGait()
{
  // Choose the gaits and parameters as selected 选择步态和参数作为选中
  switch ((int)userParameters->gait_override)
  {
  case 0:
    // Use default gait and default settings from Control code 使用来自控制代码的默认设置
    if (gaitData._currentGait != gaitData._nextGait)
    {
      createGait();
    }
    break;

  case 1:
    // Use chosen gait with default settings 使用设定步态但使用默认设置
    if (gaitData._currentGait != GaitType(userParameters->gait_type))
    {
      gaitData._nextGait = GaitType(userParameters->gait_type);
      createGait();
    }
    break;

  case 2:
    // Use chosen gait and chosen settings 使用设定步态及其参数
    // Change the gait
    if (gaitData._currentGait != GaitType(userParameters->gait_type))
    {
      gaitData._nextGait = GaitType(userParameters->gait_type);
      createGait();
    }

    // Adjust the gait parameters 调整步态参数 当前步态周期或摆动切换点和设定步态的不同
    if (fabs(gaitData.periodTimeNominal - (T)userParameters->gait_period_time) > 0.0001 ||
        fabs(gaitData.switchingPhaseNominal - (T)userParameters->gait_switching_phase) > 0.0001)
    {
      // Modify the gait with new parameters if it is overrideable 如果参数当前可覆盖 修改步态参数
      if (gaitData.overrideable == 1)
      {
        gaitData.periodTimeNominal = userParameters->gait_period_time;
        gaitData.switchingPhaseNominal = userParameters->gait_switching_phase;
        calcAuxiliaryGaitData();
      }
    }
    break;

  case 3:
    // Use NaturalGaitModification from FSM_State 使用fsm来的设定
    if (gaitData._currentGait != gaitData._nextGait)
    {
      createGait();
    }
    break;

  case 4:
    // Use NaturalGaitModification from FSM_State 使用fsm来的设定
    if (gaitData._currentGait != gaitData._nextGait)
    {
      createGait();
      period_time_natural = gaitData.periodTimeNominal;
      switching_phase_natural = gaitData.switchingPhaseNominal;
    }
    else
    {
      gaitData.periodTimeNominal = period_time_natural;
      gaitData.switchingPhaseNominal = switching_phase_natural;
      calcAuxiliaryGaitData();
    }
    break;
  }
}

/**
 * Creates the gait structure from the important defining parameters of each
 * gait 为每种创建步态通过加载的参数
 *
 * To create a standard gait you should only need to define the following:
 * 为零创建标准步态，你只需要定义如下参数
 *
 *   gaitData.periodTimeNominal
 *   gaitData.switchingPhaseNominal
 *   gaitData.phaseOffset
 *
 * The rest can be set to: 
 * 其余的可以设置为：
 *
 *   gaitData.gaitEnabled << 1, 1, 1, 1;
 *   gaitData.initialPhase = 0.0;
 *   gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
 *
 * These add flexibility to be used for very irregular gaits and transitions.
 * 这些增加了灵活性，以用于非常不规则的步态和过渡
 */
template <typename T>
void GaitScheduler<T>::createGait()
{

  std::cout << "[GAIT] Transitioning gait from " << gaitData.gaitName
            << " to ";

  // Case structure gets the appropriate parameters
  switch (gaitData._nextGait)
  {
  case GaitType::STAND:
    gaitData.gaitName = "STAND";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 10.0;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 1.0;
    gaitData.phaseOffset << 0.5, 0.5, 0.5, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 0;
    break;

  case GaitType::STAND_CYCLE:
    gaitData.gaitName = "STAND_CYCLE";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 1.0;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 1.0;
    gaitData.phaseOffset << 0.5, 0.5, 0.5, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 0;
    break;

  case GaitType::STATIC_WALK:
    gaitData.gaitName = "STATIC_WALK";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 1.25;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.8;
    gaitData.phaseOffset << 0.25, 0.0, 0.75, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;

  case GaitType::AMBLE:
    gaitData.gaitName = "AMBLE";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.5;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.6250;
    gaitData.phaseOffset << 0.0, 0.5, 0.25, 0.75;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;

  case GaitType::TROT_WALK:
    gaitData.gaitName = "TROT_WALK";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.5;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.6;
    gaitData.phaseOffset << 0.0, 0.5, 0.5, 0.0;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;

  case GaitType::TROT:
    gaitData.gaitName = "TROT";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.5;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.5;
    gaitData.phaseOffset << 0.0, 0.5, 0.5, 0.0;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;

  case GaitType::TROT_RUN:
    gaitData.gaitName = "TROT_RUN";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.4;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.4;
    gaitData.phaseOffset << 0.0, 0.5, 0.5, 0.0;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;

  case GaitType::PACE:
    gaitData.gaitName = "PACE";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.35;
    gaitData.initialPhase = 0.25;
    gaitData.switchingPhaseNominal = 0.5;
    gaitData.phaseOffset << 0.0, 0.5, 0.0, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;

  case GaitType::BOUND:
    gaitData.gaitName = "BOUND";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.4;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.4;
    gaitData.phaseOffset << 0.0, 0.0, 0.5, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;

  case GaitType::ROTARY_GALLOP:
    gaitData.gaitName = "ROTARY_GALLOP";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.4;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.2;
    gaitData.phaseOffset << 0.0, 0.8571, 0.3571, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;

  case GaitType::TRAVERSE_GALLOP:
    // TODO: find the right sequence, should be easy
    gaitData.gaitName = "TRAVERSE_GALLOP";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.5;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.2;
    gaitData.phaseOffset << 0.0, 0.8571, 0.3571, 0.5;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;

  case GaitType::PRONK:
    gaitData.gaitName = "PRONK";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    gaitData.periodTimeNominal = 0.5;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.5;
    gaitData.phaseOffset << 0.0, 0.0, 0.0, 0.0;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;

  case GaitType::THREE_FOOT:
    gaitData.gaitName = "THREE_FOOT";
    gaitData.gaitEnabled << 0, 1, 1, 1;
    gaitData.periodTimeNominal = 0.4;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = 0.666;
    gaitData.phaseOffset << 0.0, 0.666, 0.0, 0.333;
    gaitData.phaseScale << 0.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 1;
    break;

  case GaitType::CUSTOM:
    gaitData.gaitName = "CUSTOM";
    // TODO: get custom gait parameters from operator GUI
    break;

  case GaitType::TRANSITION_TO_STAND:
    gaitData.gaitName = "TRANSITION_TO_STAND";
    gaitData.gaitEnabled << 1, 1, 1, 1;
    T oldGaitPeriodTimeNominal = gaitData.periodTimeNominal;
    gaitData.periodTimeNominal = 3 * gaitData.periodTimeNominal;
    gaitData.initialPhase = 0.0;
    gaitData.switchingPhaseNominal = (gaitData.periodTimeNominal + oldGaitPeriodTimeNominal * (gaitData.switchingPhaseNominal - 1)) / gaitData.periodTimeNominal;
    gaitData.phaseOffset << (gaitData.periodTimeNominal + oldGaitPeriodTimeNominal * (gaitData.phaseVariable(0) - 1)) / gaitData.periodTimeNominal,
        (gaitData.periodTimeNominal + oldGaitPeriodTimeNominal * (gaitData.phaseVariable(1) - 1)) / gaitData.periodTimeNominal,
        (gaitData.periodTimeNominal + oldGaitPeriodTimeNominal * (gaitData.phaseVariable(2) - 1)) / gaitData.periodTimeNominal,
        (gaitData.periodTimeNominal + oldGaitPeriodTimeNominal * (gaitData.phaseVariable(3) - 1)) / gaitData.periodTimeNominal;
    gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
    gaitData.overrideable = 0;
    break;
    /*
        case GaitType::TRANSITION_FROM_STAND:
          gaitData.gaitName = "TRANSITION_FROM_STAND";
          gaitData.gaitEnabled << 1, 1, 1, 1;


          gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;

          break;
      */
  }

  // Gait has switched
  gaitData._currentGait = gaitData._nextGait;

  std::cout << gaitData.gaitName << "\n"
            << std::endl;

  // Calculate the auxilliary gait information
  calcAuxiliaryGaitData();
}

template <typename T>
void GaitScheduler<T>::calcAuxiliaryGaitData()
{

  if (gaitData.overrideable == 1)
  {
    if (userParameters->gait_override == 2)
    {
      // gaitData.periodTimeNominal = userParameters->gait_period_time;
      // gaitData.switchingPhaseNominal = userParameters->gait_switching_phase;
    }
    else if (userParameters->gait_override == 4)
    {
      //gaitData.periodTimeNominal = userParameters->gait_period_time;
      //gaitData.switchingPhaseNominal = userParameters->gait_switching_phase;
    }
  }

  // Set the gait parameters for each foot
  for (int foot = 0; foot < 4; foot++)
  {
    if (gaitData.gaitEnabled(foot) == 1)
    {
      // The scaled period time for each foot 每只脚的缩放周期时间
      gaitData.periodTime(foot) =
          gaitData.periodTimeNominal / gaitData.phaseScale(foot);

      // Phase at which to switch the foot from stance to swing //将脚从站姿转换成摇摆姿势的阶段
      gaitData.switchingPhase(foot) = gaitData.switchingPhaseNominal;

      // Initialize the phase variables according to offset 根据偏移量初始化相位变量
      gaitData.phaseVariable(foot) =
          gaitData.initialPhase + gaitData.phaseOffset(foot);

      // Find the total stance time over the gait cycle 找出整个步态周期的总站立时间
      gaitData.timeStance(foot) =
          gaitData.periodTime(foot) * gaitData.switchingPhase(foot);

      // Find the total swing time over the gait cycle 找出整个步态周期的摆动时间
      gaitData.timeSwing(foot) =
          gaitData.periodTime(foot) * (1.0 - gaitData.switchingPhase(foot));
    }
    else
    {
      // The scaled period time for each foot
      gaitData.periodTime(foot) = 0.0;

      // Phase at which to switch the foot from stance to swing
      gaitData.switchingPhase(foot) = 0.0;

      // Initialize the phase variables according to offset
      gaitData.phaseVariable(foot) = 0.0;

      // Foot is never in stance
      gaitData.timeStance(foot) = 0.0;

      // Foot is always in "swing"
      gaitData.timeSwing(foot) = 1.0 / gaitData.periodTime(foot);
    }
  }
}

/**
 * Prints relevant information about the gait and current gait state
 */
template <typename T>
void GaitScheduler<T>::printGaitInfo()
{
  // Increment printing iteration
  printIter++;

  // Print at requested frequency
  if (printIter == printNum)
  {
    std::cout << "[GAIT SCHEDULER] Printing Gait Info...\n";
    std::cout << "Gait Type: " << gaitData.gaitName << "\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "Enabled: " << gaitData.gaitEnabled(0) << " | "
              << gaitData.gaitEnabled(1) << " | " << gaitData.gaitEnabled(2)
              << " | " << gaitData.gaitEnabled(3) << "\n";
    std::cout << "Period Time: " << gaitData.periodTime(0) << "s | "
              << gaitData.periodTime(1) << "s | " << gaitData.periodTime(2)
              << "s | " << gaitData.periodTime(3) << "s\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "Contact State: " << gaitData.contactStateScheduled(0) << " | "
              << gaitData.contactStateScheduled(1) << " | "
              << gaitData.contactStateScheduled(2) << " | "
              << gaitData.contactStateScheduled(3) << "\n";
    std::cout << "Phase Variable: " << gaitData.phaseVariable(0) << " | "
              << gaitData.phaseVariable(1) << " | " << gaitData.phaseVariable(2)
              << " | " << gaitData.phaseVariable(3) << "\n";
    std::cout << "Stance Time Remaining: " << gaitData.timeStanceRemaining(0)
              << "s | " << gaitData.timeStanceRemaining(1) << "s | "
              << gaitData.timeStanceRemaining(2) << "s | "
              << gaitData.timeStanceRemaining(3) << "s\n";
    std::cout << "Swing Time Remaining: " << gaitData.timeSwingRemaining(0)
              << "s | " << gaitData.timeSwingRemaining(1) << "s | "
              << gaitData.timeSwingRemaining(2) << "s | "
              << gaitData.timeSwingRemaining(3) << "s\n";
    std::cout << std::endl;

    // Reset iteration counter
    printIter = 0;
  }
}

template class GaitScheduler<double>;
template class GaitScheduler<float>;