/*! @file GamepadCommand.h
 *  @brief The GamepadCommand type containing joystick information
 */

#ifndef PROJECT_GAMEPADCOMMAND_H
#define PROJECT_GAMEPADCOMMAND_H

#include "Utilities/utilities.h"
#include "cppTypes.h"

/*!
 * The state of the gamepad
 */
struct GamepadCommand {
  /*!
   * Construct a gamepad and set to zero.
   */
  GamepadCommand() { zero(); }

  bool leftBumper, rightBumper, leftTriggerButton, rightTriggerButton, back,
      start, a, b, x, y, leftStickButton, rightStickButton, logitechButton;

  Vec2<float> leftStickAnalog, rightStickAnalog;
  float leftTriggerAnalog, rightTriggerAnalog;

  /*!
   * Set all values to zero
   */
  void zero() {
    leftBumper = false;
    rightBumper = false;
    leftTriggerButton = false;
    rightTriggerButton = false;
    back = false;
    start = false;
    a = false;
    b = false;
    x = false;
    y = false;
    leftStickButton = false;
    rightStickButton = false;

    leftTriggerAnalog = 0;
    rightTriggerAnalog = 0;
    leftStickAnalog = Vec2<float>::Zero();
    rightStickAnalog = Vec2<float>::Zero();
  }

  /*!
   * The Logitech F310's seem to do a bad job of returning to zero exactly, so a
   * deadband around zero is useful when integrating joystick commands
   * @param f : The deadband
   */
  void applyDeadband(float f) {
    eigenDeadband(leftStickAnalog, f);
    eigenDeadband(rightStickAnalog, f);
    leftTriggerAnalog = deadband(leftTriggerAnalog, f);
    rightTriggerAnalog = deadband(rightTriggerAnalog, f);
  }

  /*!
   * Represent as human-readable string.
   * @return string representing state
   */
  std::string toString() {
    std::string result =
        "Result:\nleftBumper: " + boolToString(leftBumper) + "\n" +
        "rightBumper: " + boolToString(rightBumper) + "\n" +
        "leftTriggerButton: " + boolToString(leftTriggerButton) + "\n" +
        "rightTriggerButton: " + boolToString(rightTriggerButton) + "\n" +
        "back: " + boolToString(back) + "\n" + "start: " + boolToString(start) +
        "\n" + "a: " + boolToString(a) + "\n" + "b: " + boolToString(b) + "\n" +
        "x: " + boolToString(x) + "\n" + "y: " + boolToString(y) + "\n" +
        "leftStickButton: " + boolToString(leftStickButton) + "\n" +
        "rightStickButton: " + boolToString(rightStickButton) + "\n" +
        "leftTriggerAnalog: " + std::to_string(leftTriggerAnalog) + "\n" +
        "rightTriggerAnalog: " + std::to_string(rightTriggerAnalog) + "\n" +
        "leftStickAnalog: " + eigenToString(leftStickAnalog) + "\n" +
        "rightStickAnalog: " + eigenToString(rightStickAnalog) + "\n";
    return result;
  }
};

#endif  // PROJECT_DRIVERCOMMAND_H
