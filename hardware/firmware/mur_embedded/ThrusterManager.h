#ifndef THRUSTER_MANAGER_H
#define THRUSTER_MANAGER_H

#include "config.h"
#include <Arduino.h>
#include <Servo.h>
#include <ArduinoJson.h>

#define MILLIS_BW_COMMANDS_BEFORE_THRUSTER_SHUTDOWN 1000

class ThrusterManager {
public:
  ThrusterManager();
  void setupThrusters();
  bool processThrusters(bool message_received, DynamicJsonDocument& receiveDoc, DynamicJsonDocument& transmitDoc);

private:
  static const int NUM_SIG_PINS = 8;
  const int sig_pins[NUM_SIG_PINS] = {2, 3, 4, 5, 6, 7, 8, 9};
  Servo thruster_servos[NUM_SIG_PINS];
  float thruster_unit_commands[NUM_SIG_PINS] = {0, 0, 0, 0, 0, 0, 0, 0};
  bool thruster_unit_command_received = false;
  int thruster_us_commands[NUM_SIG_PINS] = {0, 0, 0, 0, 0, 0, 0, 0};
  bool thruster_us_command_received = false;

  const int PWM_STOP = 1500;  // Initialize/Stop
  const int PWM_MAX_FORWARD = 1900;  // Max forward
  const int PWM_MAX_REVERSE = 1100;  // Max reverse
  const float ZERO_THRESHOLD_FOR_THRUSTERS = 0.01;  // Adjust this value as needed

  elapsedMillis millis_since_last_thruster_command;

  int unit_scaled_thruster_input_to_pwm_signal(float unit_scaled_signal);
  void setThrusterCommands(const float* commands, int size);
  void setThrusterUsCommands(const int* commands, int size);
  void processReceivedThrusterCommands(DynamicJsonDocument& receiveDoc);
};

#endif // THRUSTER_MANAGER_H
