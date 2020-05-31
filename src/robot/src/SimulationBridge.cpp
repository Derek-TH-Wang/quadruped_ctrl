/*! @file SimulationBridge.cpp
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */

#include "SimulationBridge.h"
#include "Utilities/SegfaultHandler.h"
#include "Controllers/LegController.h"
// #include "rt/rt_rc_interface.h"
// #include "rt/rt_sbus.h"

/*!
 * Connect to a simulation
 */
void SimulationBridge::run() {
    printf("[Simulation Driver] Starting main loop...\n");
    // bool firstRun = true;
    for (;;) {
      runRobotControl();
    }
}

/*!
 * Run the robot controller
 */
void SimulationBridge::runRobotControl() {
  if (_firstControllerRun) {
    printf("[Simulator Driver] First run of robot controller...\n");
    try {
      _robotParams.initializeFromYamlFile("/media/derek/OS/Ubuntu/quadruped_ws/src/quadruped_robot/config/mini-cheetah-defaults.yaml");
    } catch(std::exception& e) {
      printf("Failed to initialize robot parameters from yaml file: %s\n", e.what());
      exit(1);
    }
    if(!_robotParams.isFullyInitialized()) {
      printf("Failed to initialize all robot parameters\n");
      exit(1);
    }

    auto* userControlParameters = _robotRunner->_robot_ctrl->getUserControlParameters();
    try {
      userControlParameters->initializeFromYamlFile("/media/derek/OS/Ubuntu/quadruped_ws/src/quadruped_robot/config/mc-mit-ctrl-user-parameters.yaml");
    } catch(std::exception& e) {
      printf("Failed to initialize user parameters from yaml file: %s\n", e.what());
      exit(1);
    }
    if(!userControlParameters->isFullyInitialized()) {
      printf("Failed to initialize all user parameters\n");
      exit(1);
    }
    if(userControlParameters) {
      if (userControlParameters->isFullyInitialized()) {
        printf("\tAll %ld user parameters are initialized\n",
               userControlParameters->collection._map.size());
        _simMode = SimulatorMode::RUN_CONTROLLER;
      } else {
        printf(
            "\t222but not all control parameters were initialized. Missing:\n%s\n",
            userControlParameters->generateUnitializedList().c_str());
        throw std::runtime_error(
            "not all parameters initialized when going into RUN_CONTROLLER");
      }
    } else {
      _simMode = SimulatorMode::RUN_CONTROLLER;
    }


    //delete shared mem
    // _robotRunner->driverCommand = &_sharedMemory().simToRobot.gamepadCommand;
    // _robotRunner->spiData = &_sharedMemory().simToRobot.spiData;
    // //delete ti
    // // _robotRunner->tiBoardData = _sharedMemory().simToRobot.tiBoardData;
    _robotRunner->robotType = _robot;
    //derektodo: imu callback function in this file
    // _robotRunner->vectorNavData = &_sharedMemory().simToRobot.vectorNav;
    // _robotRunner->cheaterState = &_sharedMemory().simToRobot.cheaterState;//simulation only
    // _robotRunner->spiCommand = &_sharedMemory().robotToSim.spiCommand;
    // // _robotRunner->tiBoardCommand = _sharedMemory().robotToSim.tiBoardCommand;
    _robotRunner->controlParameters = &_robotParams;
    // _robotRunner->visualizationData = &_sharedMemory().robotToSim.visualizationData;
    // _robotRunner->cheetahMainVisualization = &_sharedMemory().robotToSim.mainCheetahVisualization;

    _robotRunner->init();
    _firstControllerRun = false;
  }
  _robotRunner->run();
}
