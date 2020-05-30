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
    if (_robotParams.isFullyInitialized()) {
      printf("\tAll %ld control parameters are initialized\n",
             _robotParams.collection._map.size());
    } else {
      printf(
          "\t111but not all control parameters were initialized. Missing:\n%s\n",
          _robotParams.generateUnitializedList().c_str());
      throw std::runtime_error(
          "not all parameters initialized when going into RUN_CONTROLLER");
    }

    auto* userControlParameters = _robotRunner->_robot_ctrl->getUserControlParameters();
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
    // _robotRunner->robotType = _robot;
    //derektodo: imu callback function in this file
    // _robotRunner->vectorNavData = &_sharedMemory().simToRobot.vectorNav;
    // _robotRunner->cheaterState = &_sharedMemory().simToRobot.cheaterState;//simulation only
    // _robotRunner->spiCommand = &_sharedMemory().robotToSim.spiCommand;
    // // _robotRunner->tiBoardCommand = _sharedMemory().robotToSim.tiBoardCommand;
    // _robotRunner->controlParameters = &_robotParams;
    // _robotRunner->visualizationData = &_sharedMemory().robotToSim.visualizationData;
    // _robotRunner->cheetahMainVisualization = &_sharedMemory().robotToSim.mainCheetahVisualization;

    _robotRunner->init();
    _firstControllerRun = false;
  }
  _robotRunner->run();
}
