/*!
 * @file HardwareBridge.cpp
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */
#ifdef linux 

#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include "Configuration.h"

#include "HardwareBridge.h"
//delete rc
//#include "rt/rt_rc_interface.h"
// #include "rt/rt_sbus.h"
// #include "rt/rt_spi.h"
// #include "rt/rt_vectornav.h"
// #include "rt/rt_ethercat.h"
#include "Utilities/Utilities_print.h"

#define USE_MICROSTRAIN

/*!
 * If an error occurs during initialization, before motors are enabled, print
 * error and exit.
 * @param reason Error message string
 * @param printErrno If true, also print C errno
 */
void HardwareBridge::initError(const char* reason, bool printErrno) {
  printf("FAILED TO INITIALIZE HARDWARE: %s\n", reason);

  if (printErrno) {
    printf("Error: %s\n", strerror(errno));
  }

  exit(-1);
}

/*!
 * All hardware initialization steps that are common between Cheetah 3 and Mini Cheetah
 */
void HardwareBridge::initCommon() {
  printf("[HardwareBridge] Init stack\n");
  prefaultStack();
  printf("[HardwareBridge] Init scheduler\n");
  setupScheduler();
}

/*!
 * Writes to a 16 KB buffer on the stack. If we are using 4K pages for our
 * stack, this will make sure that we won't have a page fault when the stack
 * grows.  Also mlock's all pages associated with the current process, which
 * prevents the cheetah software from being swapped out.  If we do run out of
 * memory, the robot program will be killed by the OOM process killer (and
 * leaves a log) instead of just becoming unresponsive.
 */
void HardwareBridge::prefaultStack() {
  printf("[Init] Prefault stack...\n");
  volatile char stack[MAX_STACK_SIZE];
  memset(const_cast<char*>(stack), 0, MAX_STACK_SIZE);
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    initError(
        "mlockall failed.  This is likely because you didn't run robot as "
        "root.\n",
        true);
  }
}

/*!
 * Configures the scheduler for real time priority
 */
void HardwareBridge::setupScheduler() {
  printf("[Init] Setup RT Scheduler...\n");
  struct sched_param params;
  params.sched_priority = TASK_PRIORITY;
  if (sched_setscheduler(0, SCHED_FIFO, &params) == -1) {
    initError("sched_setscheduler failed.\n", true);
  }
}


MiniCheetahHardwareBridge::MiniCheetahHardwareBridge(RobotController* robot_ctrl, bool load_parameters_from_file)
    : HardwareBridge(robot_ctrl)//, _spiLcm(getLcmUrl(255)), _microstrainLcm(getLcmUrl(255)) { //delete lcm
{
  _load_parameters_from_file = load_parameters_from_file;
}

/*!
 * Main method for Mini Cheetah hardware
 */
void MiniCheetahHardwareBridge::run() {
  initCommon();
  initHardware();

  if(_load_parameters_from_file) {
    printf("[Hardware Bridge] Loading parameters from file...\n");

    try {
      _robotParams.initializeFromYamlFile(THIS_COM "config/mini-cheetah-defaults.yaml");
    } catch(std::exception& e) {
      printf("Failed to initialize robot parameters from yaml file: %s\n", e.what());
      exit(1);
    }

    if(!_robotParams.isFullyInitialized()) {
      printf("Failed to initialize all robot parameters\n");
      exit(1);
    }

    printf("Loaded robot parameters\n");

    if(_userControlParameters) {
      try {
        _userControlParameters->initializeFromYamlFile(THIS_COM "config/mc-mit-ctrl-user-parameters.yaml");
      } catch(std::exception& e) {
        printf("Failed to initialize user parameters from yaml file: %s\n", e.what());
        exit(1);
      }

      if(!_userControlParameters->isFullyInitialized()) {
        printf("Failed to initialize all user parameters\n");
        exit(1);
      }

      printf("Loaded user parameters\n");
    } else {
      printf("Did not load user parameters because there aren't any\n");
    }
  } else {
    printf("[Hardware Bridge] Loading parameters over LCM...\n");
    while (!_robotParams.isFullyInitialized()) {
      printf("[Hardware Bridge] Waiting for robot parameters...\n");
      usleep(1000000);
    }

    if(_userControlParameters) {
      while (!_userControlParameters->isFullyInitialized()) {
        printf("[Hardware Bridge] Waiting for user parameters...\n");
        usleep(1000000);
      }
    }
  }



  printf("[Hardware Bridge] Got all parameters, starting up!\n");

  _robotRunner =
      new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");

  //delete gamepad
  // _robotRunner->driverCommand = &_gamepadCommand;
  //delete spi
  // _robotRunner->spiData = &_spiData;
  // _robotRunner->spiCommand = &_spiCommand;
  _robotRunner->robotType = RobotType::MINI_CHEETAH;
  _robotRunner->vectorNavData = &_vectorNavData;
  _robotRunner->controlParameters = &_robotParams;
  // _robotRunner->visualizationData = &_visualizationData;
  // _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;

  _firstRun = false;

  // init control thread

  statusTask.start();


  // robot controller start
  _robotRunner->start();


  for (;;) {
    usleep(1000000);
    // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
  }
}


/*!
 * Initialize Mini Cheetah specific hardware
 */
void MiniCheetahHardwareBridge::initHardware() {
  _vectorNavData.quat << 1, 0, 0, 0;
#ifndef USE_MICROSTRAIN
  printf("[MiniCheetahHardware] Init vectornav\n");
  if (!init_vectornav(&_vectorNavData)) {
    printf("Vectornav failed to initialize\n");
    //initError("failed to initialize vectornav!\n", false);
  }
#endif

  //delete spi
  // init_spi();
  //delete imu
  // _microstrainInit = _microstrainImu.tryInit(0, 921600);
}





#endif
