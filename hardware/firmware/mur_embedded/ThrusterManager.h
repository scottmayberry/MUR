// ThrusterManager.h
// Header file for the ThrusterManager class.
// Manages thruster initialization, command processing, and PWM signal generation
// for the MUR underwater robot's compute and battery fuselage boards.

#ifndef THRUSTER_MANAGER_H
#define THRUSTER_MANAGER_H

#include "config.h"              // Include configuration settings
#include <Arduino.h>             // Include Arduino core library
#include <Servo.h>               // Include Servo library for PWM control
#include <ArduinoJson.h>         // Include ArduinoJson library for JSON handling

// Define the timeout threshold in milliseconds before thrusters are automatically shut down
#define MILLIS_BW_COMMANDS_BEFORE_THRUSTER_SHUTDOWN 1000 // 1 second

class ThrusterManager {
public:
  // Constructor: Initializes the ThrusterManager
  ThrusterManager();

  // Sets up all thrusters by configuring their pins and initializing servos
  void setupThrusters();

  // Processes thruster commands based on received messages and manages thruster states
  // Returns true if thrusters are active and data is ready to transmit
  bool processThrusters(bool message_received, DynamicJsonDocument& receiveDoc, DynamicJsonDocument& transmitDoc);

private:
  static const int NUM_SIG_PINS = 8; // Number of thruster signal pins

  // Array of signal pins connected to thrusters
  const int sig_pins[NUM_SIG_PINS] = {2, 3, 4, 5, 6, 7, 8, 9};

  Servo thruster_servos[NUM_SIG_PINS]; // Array of Servo objects controlling each thruster

  // Arrays to hold thruster commands
  float thruster_unit_commands[NUM_SIG_PINS] = {0, 0, 0, 0, 0, 0, 0, 0}; // Unit-scaled commands
  int thruster_us_commands[NUM_SIG_PINS] = {0, 0, 0, 0, 0, 0, 0, 0};     // Microsecond-based commands

  // Flags indicating whether new thruster commands have been received
  bool thruster_unit_command_received = false;
  bool thruster_us_command_received = false;

  // Constants defining PWM signal ranges
  const int PWM_STOP = 1500;           // Neutral PWM value indicating no thrust
  const int PWM_MAX_FORWARD = 1900;    // Maximum forward thrust PWM value
  const int PWM_MAX_REVERSE = 1100;    // Maximum reverse thrust PWM value

  // Threshold below which thrusters are considered to be in a neutral state
  const float ZERO_THRESHOLD_FOR_THRUSTERS = 0.01; // Adjust this value as needed

  elapsedMillis millis_since_last_thruster_command; // Timer tracking time since last thruster command

  // Converts unit-scaled thruster input to a PWM signal value
  int unit_scaled_thruster_input_to_pwm_signal(float unit_scaled_signal);

  // Sets thruster commands based on unit-scaled input
  void setThrusterCommands(const float* commands, int size);

  // Sets thruster commands based on direct microsecond input
  void setThrusterUsCommands(const int* commands, int size);

  // Processes received thruster commands from the JSON document
  void processReceivedThrusterCommands(DynamicJsonDocument& receiveDoc);
};

#endif // THRUSTER_MANAGER_H
