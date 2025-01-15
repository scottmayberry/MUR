// ThrusterManager.cpp
// Implementation of the ThrusterManager class for managing thrusters
// on the MUR underwater robot's compute and battery fuselage boards.

#include "config.h"               // Include configuration settings
#include "ThrusterManager.h"     // Include the ThrusterManager class definition

// Constructor for the ThrusterManager class.
// Initializes any necessary member variables.
ThrusterManager::ThrusterManager() {}

// Sets up all thrusters by configuring their respective pins and initializing servos.
void ThrusterManager::setupThrusters() {
  // Iterate through each thruster signal pin
  for (int i = 0; i < NUM_SIG_PINS; i++) {
    pinMode(sig_pins[i], OUTPUT);            // Set the signal pin as an output
    thruster_servos[i].attach(sig_pins[i]);  // Attach the servo to the signal pin
    Serial.print("Thruster ");               // Print thruster number for debugging
    Serial.print(i);
    Serial.println(" starting up.");
    
    // Convert the initial thruster command to a PWM signal
    int pwmValue = unit_scaled_thruster_input_to_pwm_signal(thruster_us_commands[i]);
    thruster_servos[i].writeMicroseconds(pwmValue); // Send the PWM signal to the thruster
    delay(3000); // Wait for 3 seconds to allow thruster to stabilize
  }
}

// Processes thruster commands based on received messages and manages thruster states.
// Returns true if thrusters are active and data is ready to transmit.
bool ThrusterManager::processThrusters(bool message_received, DynamicJsonDocument& receiveDoc, DynamicJsonDocument& transmitDoc) {
  if(message_received) {
    processReceivedThrusterCommands(receiveDoc); // Process incoming thruster commands if a message is received
  }
  
  // Shutdown thrusters if no command has been received within the specified timeframe
  if (millis_since_last_thruster_command >= MILLIS_BW_COMMANDS_BEFORE_THRUSTER_SHUTDOWN) {
    // Iterate through each thruster and set PWM to 0 (stop)
    for (int i = 0; i < NUM_SIG_PINS; i++) {
      int pwmValue = unit_scaled_thruster_input_to_pwm_signal(0); // Convert zero input to PWM signal
      thruster_servos[i].writeMicroseconds(pwmValue);              // Send stop signal to thruster
    }
    millis_since_last_thruster_command = 0; // Reset the command timer
    Serial.print("Thruster Millis Threshold: ");
    Serial.println(millis()); // Print the current time for debugging
    transmitDoc["isThrust"] = false; // Indicate in JSON that thrusters are inactive
    return false; // Indicate that thrusters are not active
  }

  // If no thruster commands have been received, do not process further
  if (!thruster_us_command_received && !thruster_unit_command_received) {
    return false; // Indicate that thrusters are not active
  }

  // Write thruster commands based on the type of command received (unit or microseconds)
  if (thruster_unit_command_received) {
    // Process unit-scaled thruster commands
    for (int i = 0; i < NUM_SIG_PINS; i++) {
      int pwmValue = unit_scaled_thruster_input_to_pwm_signal(thruster_unit_commands[i]); // Convert unit input to PWM signal
      thruster_servos[i].writeMicroseconds(pwmValue); // Send PWM signal to thruster
    }
  } else {
    // Process thruster commands received in microseconds
    for (int i = 0; i < NUM_SIG_PINS; i++) {
      thruster_servos[i].writeMicroseconds(thruster_us_commands[i]); // Send PWM signal directly
    }
  }

  // Reset the command flags and timer after processing
  thruster_unit_command_received = false;
  thruster_us_command_received = false;
  millis_since_last_thruster_command = 0;

  return true; // Indicate that thrusters are active and data is ready to transmit
}

// Sets thruster commands based on unit-scaled input.
// Copies the provided commands into the thruster_unit_commands array.
void ThrusterManager::setThrusterCommands(const float* commands, int size) {
  if (size > NUM_SIG_PINS) return; // Prevent buffer overflow by ensuring size does not exceed number of thrusters
  for (int i = 0; i < size; i++) {
    thruster_unit_commands[i] = commands[i]; // Assign each command to the corresponding thruster
  }
  thruster_unit_command_received = true; // Flag that unit commands have been received
  millis_since_last_thruster_command = 0; // Reset the command timer
}

// Sets thruster commands based on direct microsecond input.
// Copies the provided commands into the thruster_us_commands array.
void ThrusterManager::setThrusterUsCommands(const int* commands, int size) {
  if (size > NUM_SIG_PINS) return; // Prevent buffer overflow by ensuring size does not exceed number of thrusters
  for (int i = 0; i < size; i++) {
    thruster_us_commands[i] = commands[i]; // Assign each command to the corresponding thruster
  }
  thruster_us_command_received = true; // Flag that microsecond commands have been received
  millis_since_last_thruster_command = 0; // Reset the command timer
}

// Converts unit-scaled thruster input to a PWM signal value.
// Applies a non-linear transformation to map unit inputs to PWM ranges suitable for thrusters.
int ThrusterManager::unit_scaled_thruster_input_to_pwm_signal(float unit_scaled_signal) {
  if (abs(unit_scaled_signal) < ZERO_THRESHOLD_FOR_THRUSTERS) {
    return 1500; // Neutral PWM value indicating no thrust
  } else {
    int out = 0;
    float calc = 3.5 * abs(unit_scaled_signal); // Calculate intermediate value based on unit input
    if (unit_scaled_signal > 0) {
      // Forward thrust calculation
      out = int(round(-10.8 * sq(calc) + calc * 119.5 + 1540));
    } else {
      // Reverse thrust calculation
      out = int(round(11.31 * sq(calc) + calc * -137.4 + 1456));
    }
    return out; // Return the calculated PWM value
}

// Processes received thruster commands from the JSON document.
// Updates thruster command arrays based on the received data.
void ThrusterManager::processReceivedThrusterCommands(DynamicJsonDocument& receiveDoc) {
    // Get a reference to the root object of the JSON document
    JsonObject obj = receiveDoc.as<JsonObject>();

    // Process thruster commands received in microseconds
    if (obj.containsKey("thruster_us") && obj["thruster_us"].is<JsonArray>()) {
        thruster_us_command_received = true; // Flag that microsecond commands have been received

        JsonArray thrusterArray = obj["thruster_us"]; // Get the array of thruster commands

        // Loop through the array and assign values to thruster_us_commands
        for (size_t i = 0; i < thrusterArray.size() && i < NUM_SIG_PINS; i++) {
            thruster_us_commands[i] = thrusterArray[i].as<int>(); // Assign each command
        }

        // Optional: Print the received thruster commands for debugging
        Serial.print("Received thruster us commands: ");
        for (size_t i = 0; i < thrusterArray.size() && i < NUM_SIG_PINS; i++) {
            Serial.print(thruster_us_commands[i]); // Print each command
            if (i < 5) {            // Adjusted condition to avoid trailing comma
                Serial.print(", ");
            }
        }
        Serial.println(); // Newline for clarity
    }

    // Process thruster commands received in unit-scaled format
    if (obj.containsKey("thruster_unit") && obj["thruster_unit"].is<JsonArray>()) {
        thruster_unit_command_received = true; // Flag that unit commands have been received

        JsonArray thrusterArray = obj["thruster_unit"]; // Get the array of thruster commands

        // Loop through the array and assign values to thruster_unit_commands
        for (size_t i = 0; i < thrusterArray.size() && i < NUM_SIG_PINS; i++) {
            thruster_unit_commands[i] = thrusterArray[i].as<float>(); // Assign each command
        }

        // Optional: Print the received thruster commands for debugging
        Serial.print("Received thruster unit commands: ");
        for (size_t i = 0; i < thrusterArray.size() && i < NUM_SIG_PINS; i++) {
            Serial.print(thruster_unit_commands[i], 3); // Print each command with 3 decimal places
            if (i < 5) {                  // Adjusted condition to avoid trailing comma
                Serial.print(", ");
            }
        }
        Serial.println(); // Newline for clarity
    }
}
