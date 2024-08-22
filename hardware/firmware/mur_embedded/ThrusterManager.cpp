#include "config.h"
#include "ThrusterManager.h"

ThrusterManager::ThrusterManager() {}

void ThrusterManager::setupThrusters() {
  for (int i = 0; i < NUM_SIG_PINS; i++) {
    pinMode(sig_pins[i], OUTPUT);
    thruster_servos[i].attach(sig_pins[i]);
    Serial.print("Thruster ");
    Serial.print(i);
    Serial.println(" starting up.");
    int pwmValue = unit_scaled_thruster_input_to_pwm_signal(thruster_us_commands[i]);
    thruster_servos[i].writeMicroseconds(pwmValue);
    delay(3000);
  }
}

bool ThrusterManager::processThrusters(bool message_received, DynamicJsonDocument& receiveDoc, DynamicJsonDocument& transmitDoc) {
  if(message_received) {processReceivedThrusterCommands(receiveDoc);}
  // Shutdown thrusters if command hasn't been received in a certain timeframe

  if (millis_since_last_thruster_command >= MILLIS_BW_COMMANDS_BEFORE_THRUSTER_SHUTDOWN) {
    for (int i = 0; i < NUM_SIG_PINS; i++) {
      int pwmValue = unit_scaled_thruster_input_to_pwm_signal(0);
      thruster_servos[i].writeMicroseconds(pwmValue);
    }
    millis_since_last_thruster_command = 0;
    Serial.print("Thruster Millis Threshold: ");
    Serial.println(millis());
    transmitDoc["isThrust"] = false;
    return false;
  }

  if (!thruster_us_command_received && !thruster_unit_command_received) {
    return false;
  }

  // Write thruster commands, either unit or us
  if (thruster_unit_command_received) {
    for (int i = 0; i < NUM_SIG_PINS; i++) {
      int pwmValue = unit_scaled_thruster_input_to_pwm_signal(thruster_unit_commands[i]);
      thruster_servos[i].writeMicroseconds(pwmValue);
    }
  } else {
    for (int i = 0; i < NUM_SIG_PINS; i++) {
      thruster_servos[i].writeMicroseconds(thruster_us_commands[i]);
    }
  }

  // Reset the flag after processing
  thruster_unit_command_received = false;
  thruster_us_command_received = false;
  millis_since_last_thruster_command = 0;

  return true;
}

void ThrusterManager::setThrusterCommands(const float* commands, int size) {
  if (size > NUM_SIG_PINS) return;
  for (int i = 0; i < size; i++) {
    thruster_unit_commands[i] = commands[i];
  }
  thruster_unit_command_received = true;
  millis_since_last_thruster_command = 0;
}

void ThrusterManager::setThrusterUsCommands(const int* commands, int size) {
  if (size > NUM_SIG_PINS) return;
  for (int i = 0; i < size; i++) {
    thruster_us_commands[i] = commands[i];
  }
  thruster_us_command_received = true;
  millis_since_last_thruster_command = 0;
}

int ThrusterManager::unit_scaled_thruster_input_to_pwm_signal(float unit_scaled_signal) {
  if (abs(unit_scaled_signal) < ZERO_THRESHOLD_FOR_THRUSTERS) {
    return 1500;
  } else {
    int out = 0;
    float calc = 3.5 * abs(unit_scaled_signal);
    if (unit_scaled_signal > 0) {
      out = int(round(-10.8 * sq(calc) + calc * 119.5 + 1540));
    } else {
      out = int(round(11.31 * sq(calc) + calc * -137.4 + 1456));
    }
    return out;
  }
}

void ThrusterManager::processReceivedThrusterCommands(DynamicJsonDocument& receiveDoc) {
    // Get a reference to the root object
    JsonObject obj = receiveDoc.as<JsonObject>();

    // thruster us commands
    // Check if the received JSON contains the "thruster" key and is an array
    if (obj.containsKey("thruster_us") && obj["thruster_us"].is<JsonArray>()) {
        // Set the thruster_command_received flag to true
        thruster_us_command_received = true;

        JsonArray thrusterArray = obj["thruster_us"];
        
        // Loop through the array and store the values in the thruster_commands array
        for (size_t i = 0; i < thrusterArray.size() && i < NUM_SIG_PINS; i++) {
        thruster_us_commands[i] = thrusterArray[i].as<int>();
        }
        

        // Optional: Print the received thruster commands for debugging
        Serial.print("Received thruster us commands: ");
        for (size_t i = 0; i < thrusterArray.size() && i < NUM_SIG_PINS; i++) {
        Serial.print(thruster_us_commands[i]);  // Print with 3 decimal places
        if (i < 5) {
            Serial.print(", ");
        }
        }
        Serial.println();
    }

    // thruster unit commands
    if (obj.containsKey("thruster_unit") && obj["thruster_unit"].is<JsonArray>()) {
        // Set the thruster_command_received flag to true
        thruster_unit_command_received = true;
        
        JsonArray thrusterArray = obj["thruster_unit"];
        
        // Loop through the array and store the values in the thruster_commands array
        for (size_t i = 0; i < thrusterArray.size() && i < NUM_SIG_PINS; i++) {
        thruster_unit_commands[i] = thrusterArray[i].as<float>();
        }

        // Optional: Print the received thruster commands for debugging
        Serial.print("Received thruster unit commands: ");
        for (size_t i = 0; i < thrusterArray.size() && i < NUM_SIG_PINS; i++) {
        Serial.print(thruster_unit_commands[i], 3);  // Print with 3 decimal places
        if (i < 5) {
            Serial.print(", ");
        }
        }
        Serial.println();
    }
}
