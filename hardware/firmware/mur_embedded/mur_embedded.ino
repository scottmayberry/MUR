// mur_embedded.ino
// Arduino Sketch for the MUR (Miniature Underwater Robot)
// Manages Ethernet connectivity, sensor data collection, and thruster control
// based on the board type (Compute or Fuselage).

#include "config.h"                // Include configuration settings
#include "EthernetManager.h"       // Include EthernetManager class
#include "SensorManager.h"         // Include SensorManager class
#include "ThrusterManager.h"       // Include ThrusterManager class


// Instantiate the EthernetManager with the defined Ethernet CS pin
EthernetManager ethernetManager(CS_ETH_PIN);

// Instantiate the SensorManager to handle all sensor-related functionalities
SensorManager sensorManager;

// Conditionally instantiate the ThrusterManager only if the board type is Fuselage
#ifdef BOARD_FUSELAGE
ThrusterManager thrusterManager;
#endif

/**
 * @brief Prints the capacity of the transmit and receive JSON documents.
 *
 * @param transmitDoc The JSON document used for transmitting data.
 * @param receiveDoc The JSON document used for receiving data.
 */
void print_json_capacity(DynamicJsonDocument& transmitDoc, DynamicJsonDocument& receiveDoc){
    Serial.print("Transmit Doc Capacity: ");
    Serial.print(TRANSMIT_JSON_SIZE);
    Serial.print("/");
    Serial.println(transmitDoc.capacity());

    Serial.print("Receive Doc Capacity: ");
    Serial.print(RECEIVE_JSON_SIZE);
    Serial.print("/");
    Serial.println(receiveDoc.capacity());
}

/**
 * @brief The setup function runs once when the microcontroller is powered on or reset.
 * Initializes serial communication, SPI, Ethernet, sensors, and thrusters.
 */
void setup() {
    Serial.begin(115200);          // Initialize serial communication at 115200 baud rate
    SPI.begin();                   // Initialize SPI communication
    delay(500);                    // Wait for 500 milliseconds

    // Uncomment the following line to print JSON document capacities for debugging
    // print_json_capacity();

    delay(100);                    // Short delay before starting Ethernet
    startEthernet();               // Initialize Ethernet connectivity
    delay(100);                    // Short delay after Ethernet setup

    sensorManager.setupSensors();  // Initialize and set up all sensors
    delay(100);                    // Short delay after sensor setup

    #ifdef BOARD_FUSELAGE
    thrusterManager.setupThrusters(); // Initialize and set up all thrusters (Fuselage board only)
    #endif

    Serial.println("Starting MUR"); // Print a startup message to the serial monitor
    delay(100);                      // Short delay after startup message
}

/**
 * @brief Initializes Ethernet connectivity.
 * Attempts to start Ethernet and retries until successful.
 */
void startEthernet(){
    // Continuously attempt to initialize Ethernet until successful
    while (!ethernetManager.begin()) {
        Serial.println("Failed to initialize Ethernet. Retrying."); // Print failure message
        delay(1000); // Wait for 1 second before retrying
    }
}

/**
 * @brief The loop function runs repeatedly after the setup function.
 * Handles incoming messages, processes sensor data, and controls thrusters.
 */
void loop() {
    delay(1); // Minimal delay to prevent overwhelming the CPU

    // Flags to determine if data is ready to transmit or if a message has been received
    bool ready_to_transmit = false;
    bool received_message = false;

    // Create JSON documents for transmitting and receiving data
    DynamicJsonDocument transmitDoc(TRANSMIT_JSON_SIZE);
    DynamicJsonDocument receiveDoc(RECEIVE_JSON_SIZE);

    // Uncomment the following line to print JSON document capacities for debugging
    // print_json_capacity(transmitDoc, receiveDoc);

    // Check if the server IP has been set before attempting communication
    if (ethernetManager.isServerIPSet())
    {
        // Process incoming Ethernet messages and handle any received commands
        received_message = ethernetManager.processEthernetReceiving(receiveDoc);
        
        #ifdef BOARD_FUSELAGE
        // If on the Fuselage board, process thruster commands based on received messages
        thrusterManager.processThrusters(received_message, receiveDoc, transmitDoc);
        #endif

        // Collect sensor data and add it to the transmit JSON document
        ready_to_transmit = sensorManager.processSensors(transmitDoc);
        
        if(ready_to_transmit)
        {
            // If data is ready, send the JSON document to the server via UDP
            ethernetManager.sendUDPMessageToServer(transmitDoc);
        }
    } 
    else 
    {
        // If the server IP is not set, continue processing incoming Ethernet messages
        ethernetManager.processEthernetReceiving(receiveDoc);
        
        // If the server IP is obtained after processing, reset sensor manager timers
        if (ethernetManager.isServerIPSet()){
            sensorManager.reset_timers();
        }
    }

    // Periodically broadcast a request for the server IP if the time threshold is met
    ethernetManager.broadcastServerIpRequestIfTimeMet();

    // Clear the JSON documents to free up memory for the next iteration
    transmitDoc.clear();
    receiveDoc.clear();
}
