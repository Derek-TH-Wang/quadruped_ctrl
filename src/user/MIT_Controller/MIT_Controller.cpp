#include "MIT_Controller.hpp"

MIT_Controller::MIT_Controller() : RobotController() {}

//#define RC_ESTOP
/**
 * Initializes the Control FSM.
 */
void MIT_Controller::initializeController() {
  // Initialize a new GaitScheduler object, dt = 5ms
  _gaitScheduler = new GaitScheduler<float>(&userParameters,
                                            _controlParameters->controller_dt);

  // Initializes the Control FSM with all the required data
  _controlFSM = new ControlFSM<float>(
      _quadruped, _stateEstimator, _legController, _gaitScheduler,
      _desiredStateCommand, _controlParameters,  //_visualizationData,
      &userParameters);
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void MIT_Controller::runController() {
  // Calculate gait phase state in realtime
  _gaitScheduler->step();

  // Find the desired state trajectory according to gamepad input vel
  _desiredStateCommand->convertToStateCommands();

  // Run the Control FSM code
  _controlFSM->runFSM();
}
