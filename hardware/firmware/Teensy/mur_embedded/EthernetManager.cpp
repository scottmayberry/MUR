// EthernetManager.cpp
// Implementation of the EthernetManager class for managing Ethernet connectivity
// on a MUR's compute and battery fuselage boards.

#include "config.h"               // Include configuration settings
#include "EthernetManager.h"      // Include the EthernetManager class definition

// Constructor for the EthernetManager class.
// Initializes the chip select (CS) pin used for SPI communication.
EthernetManager::EthernetManager(int csPin)
  : _csPin(csPin) {}              // Initialize the _csPin member variable

// Initializes the EthernetManager.
// Sets up Ethernet, SPI, network configurations, and establishes initial connections.
bool EthernetManager::begin() {
  // Initialize the Ethernet module with the specified CS pin
  Ethernet.init(_csPin);
  
  // Start the SPI bus
  SPI.begin();

  // Convert the MAC address to a human-readable string and print it
  convertMACToString();
  Serial.print("MAC: ");
  Serial.println(_mac_char_arr);
  
  // Reset the W5500 Ethernet controller to ensure it's in a known state
  resetW5500();

  // Attempt to initialize Ethernet using DHCP
  if (!initializeEthernet()) {
    Serial.println("Failed to initialize Ethernet");
    return false;  // Return false if Ethernet initialization fails
  }

  // Retrieve the subnet mask from the Ethernet module
  IPAddress subnetMask = Ethernet.subnetMask();
  
  // Get the local IP address assigned by DHCP
  _localIP = Ethernet.localIP();
  
  // Calculate the broadcast IP address based on local IP and subnet mask
  calculateBroadcastAddress(_localIP, subnetMask, _broadcastIP);
  Serial.print("Broadcast IP: \t");
  Serial.println(_broadcastIP);

  // Print the network configuration details
  Serial.print("My IP address: \t");
  Serial.println(Ethernet.localIP());

  Serial.print("Subnet Mask: \t");
  Serial.println(Ethernet.subnetMask());

  Serial.print("Gateway IP: \t");
  Serial.println(Ethernet.gatewayIP());

  Serial.print("DNS Server IP: \t");
  Serial.println(Ethernet.dnsServerIP());

  // Attempt to connect to the server designated for router updates
  Serial.print("Connecting to ");
  Serial.print(_server_for_router_update);
  Serial.println("...");

  EthernetClient client;  // Create an Ethernet client instance

  // Retry connecting to the server up to 10 times
  for(int retry = 0; retry < 10; retry++){
    // Attempt to connect to the server on port 80 (HTTP)
    if (client.connect(_server_for_router_update, 80)) {
      Serial.print("Connected to ");
      Serial.println(client.remoteIP());
      
      // Send an HTTP GET request to the server
      client.println("GET /search?q=arduino HTTP/1.1");
      client.println("Host: www.example.com");
      client.println("Connection: close");
      client.println();  // End of HTTP request
      break;  // Exit the retry loop upon successful connection
    } else {
      // If connection fails, print an error message and wait before retrying
      Serial.println("Connection failed");
      delay(1000);  // Wait for 1 second before retrying
    }
  }

  // Begin listening for UDP packets on the communication port
  _udp.begin(COMM_PORT);
  
  // Reset the timer tracking successful UDP reads
  millis_since_last_successful_udp_read = 0;
  
  return true;  // Return true indicating successful initialization
}

// Sends a UDP message to the server with the provided JSON document.
void EthernetManager::sendUDPMessageToServer(DynamicJsonDocument& transmitDoc) {
  transmitDoc["id"] = BOARD_ID;  // Add the board ID to the JSON document
  
  _udp.beginPacket(_serverIP, COMM_PORT);  // Begin a UDP packet to the server's IP and port
  serializeJson(transmitDoc, _udp);        // Serialize the JSON document into the UDP packet
  _udp.endPacket();                        // End and send the UDP packet
  
  delay(1);  // Short delay to ensure the packet is sent
}

// Broadcasts a server IP request if the specified time threshold has been met.
void EthernetManager::broadcastServerIpRequestIfTimeMet() {
  // Check if the time since the last broadcast ping is less than the threshold
  if(millis_since_last_broadcast_ping < MILLIS_THRESHOLD_BROADCAST_PING_RATE){
    return;  // Exit the function if it's not time to broadcast yet
  }
  
  millis_since_last_broadcast_ping = 0;  // Reset the broadcast ping timer
  
  DynamicJsonDocument transmitDoc(TRANSMIT_JSON_SIZE);  // Create a JSON document for transmission
  transmitDoc["id"] = BOARD_ID;                        // Add the board ID
  transmitDoc["serverRequestForIP"] = _localIP;        // Request the server IP using local IP
  transmitDoc["mac"] = _mac_char_arr;                  // Include the MAC address
  transmitDoc["BROADCAST_PORT"] = BROADCAST_PORT;      // Specify the broadcast port
  transmitDoc["COMM_PORT"] = COMM_PORT;                // Specify the communication port
  
  // If the server IP has been set, include it in the broadcast
  if(isServerIPSet()){
    transmitDoc["serverIP"] = _serverIP;
  }

  _udp.beginPacket(_broadcastIP, BROADCAST_PORT);  // Begin a UDP packet to the broadcast IP and port
  serializeJson(transmitDoc, _udp);                // Serialize the JSON document into the UDP packet
  _udp.endPacket();                                // End and send the UDP packet
  
  delay(1);  // Short delay to ensure the packet is sent
}

// Checks if the server IP has been set.
bool EthernetManager::isServerIPSet(){
  return _serverIP_set;  // Return the status of the server IP flag
}

// Processes incoming Ethernet data and deserializes JSON if applicable.
bool EthernetManager::processEthernetReceiving(DynamicJsonDocument& receiveDoc) {
  int packetSize = _udp.parsePacket();  // Check for incoming UDP packets
  
  if (packetSize) {  // If a packet is received
    millis_since_last_successful_udp_read = 0;  // Reset the UDP read timer
    
    // Check if the incoming packet size exceeds the buffer size
    if (packetSize >= RECEIVE_JSON_SIZE) {
      Serial.print("ERROR PACKET TOO LARGE. DISCARDING...");
      
      // Allocate a buffer to discard the oversized packet
      char discardBuffer[packetSize];
      _udp.read(discardBuffer, packetSize);  // Read and discard the packet
      Serial.println("...Discard complete");
      delay(1);  // Short delay
      return false;  // Indicate that processing failed due to oversized packet
    }

    // Read the incoming packet into the packetBuffer
    _udp.read(packetBuffer, RECEIVE_JSON_SIZE);
    packetBuffer[packetSize] = '\0';  // Null-terminate the buffer for string operations

    Serial.println(packetBuffer);  // Print the received packet for debugging

    // Check if the packet matches the server identifier string
    if (strcmp(packetBuffer, _serverIdentifierCharArr) == 0){
        _serverIP = _udp.remoteIP();  // Set the server IP to the remote IP of the packet
        _serverIP_set = true;          // Flag that the server IP has been set
        Serial.print("Server IP is ");
        Serial.println(_serverIP);
        delay(1);  // Short delay
        return false;  // Indicate that no further processing is needed
    }
    
    // Look for the start of a JSON object ('{' or '[') within the first 8 bytes
    int startIdx = -1;
    for (int i = 0; i < 8 && i < packetSize; i++) {
      if (packetBuffer[i] == '{' || packetBuffer[i] == '[') {
        startIdx = i;  // Record the index where JSON starts
        break;
      }
    }

    // If no JSON start character is found, ignore the packet
    if (startIdx == -1) {
      delay(1);  // Short delay
      return false;  // Indicate that no valid JSON was found
    }

    receiveDoc.clear();  // Clear the JSON document before deserialization
    
    // Attempt to deserialize the JSON starting from the detected JSON start index
    DeserializationError error = deserializeJson(receiveDoc, packetBuffer + startIdx);
    
    if (error) {  // If deserialization fails
      receiveDoc.clear();  // Clear the JSON document
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());  // Print the error message
      delay(1);  // Short delay
      return false;  // Indicate that deserialization failed
    }
    
    // At this point, receiveDoc contains the deserialized JSON data
    // You can add further processing of receiveDoc here as needed
    
    delay(1);  // Short delay
    return true;  // Indicate successful processing of received data
  } else{
    // If no packet is received, check for UDP timeout and reset if necessary
    timeoutCheckAndUDPReset();
  }
  
  delay(1);  // Short delay
  return false;  // Indicate that no data was processed
}

// Checks for UDP read timeouts and resets the UDP connection if the threshold is exceeded.
void EthernetManager::timeoutCheckAndUDPReset() {
  // Check if the time since the last successful UDP read exceeds the reset threshold
  if(millis_since_last_successful_udp_read >= MILLIS_THRESHOLD_BETWEEN_UDP_READS_TO_RESET){
    millis_since_last_successful_udp_read = 0;  // Reset the UDP read timer
    Serial.print("Resetting UDP: ");
    Serial.println(millis());  // Print the current time for debugging
    
    _udp.stop();  // Stop the current UDP connection
    delay(2);      // Short delay to ensure the UDP stop completes
    
    _udp.begin(COMM_PORT);  // Restart the UDP connection on the communication port
    delay(2);               // Short delay to ensure the UDP start completes
  }
}

// Resets the W5500 Ethernet controller via SPI communication.
void EthernetManager::resetW5500() {
  // Begin an SPI transaction with specified settings (14MHz, MSB first, SPI mode 0)
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  
  digitalWrite(_csPin, LOW);    // Select the W5500 chip by setting CS low
  SPI.transfer(0x00);           // Send a reset command byte (specific to W5500)
  SPI.transfer(0x80);           // Send another reset command byte
  digitalWrite(_csPin, HIGH);   // Deselect the W5500 chip by setting CS high
  
  SPI.endTransaction();          // End the SPI transaction
  delay(200);                    // Wait for 200ms to allow the reset to complete
}

// Initializes Ethernet using DHCP, retrying up to 5 times if necessary.
bool EthernetManager::initializeEthernet() {
  Serial.println("Initialize Ethernet with DHCP:");
  
  for (int retry = 0; retry < 5; retry++) {  // Attempt DHCP initialization up to 5 times
    if (Ethernet.begin(_mac) != 0) {        // Attempt to begin Ethernet with the provided MAC
      Serial.print("  DHCP assigned IP ");
      Serial.println(Ethernet.localIP());    // Print the assigned IP address
      return true;                           // Return true if DHCP was successful
    } else {
      Serial.println("Failed to configure Ethernet using DHCP. Retrying...");
      
      // Check if the Ethernet hardware is present
      if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found. Sorry, can't run without hardware. :(");
        return false;  // Return false if Ethernet hardware is not found
      }
      
      // Check if the Ethernet link is active
      if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected.");
      }
      
      resetW5500();               // Reset the W5500 Ethernet controller
      digitalWrite(LED_PIN, HIGH);  // Turn on an LED to indicate a reset (assuming LED_PIN is defined)
      delay(1000);                  // Wait for 1 second
      digitalWrite(LED_PIN, LOW);   // Turn off the LED
    }
  }
  
  return false;  // Return false if all DHCP initialization attempts fail
}

// Calculates the broadcast IP address based on the local IP and subnet mask.
void EthernetManager::calculateBroadcastAddress(IPAddress localIP, IPAddress subnetMask, IPAddress &broadcastIP) {
  for (int i = 0; i < 4; i++) {
    broadcastIP[i] = localIP[i] | ~subnetMask[i];  // Perform bitwise OR between local IP and inverted subnet mask
  }
}

// Converts the MAC address byte array to a human-readable string format.
void EthernetManager::convertMACToString() {
    snprintf(_mac_char_arr, sizeof(_mac_char_arr), "%02X:%02X:%02X:%02X:%02X:%02X", 
             _mac[0], _mac[1], _mac[2], _mac[3], _mac[4], _mac[5]);
    // Formats the MAC address as "XX:XX:XX:XX:XX:XX" and stores it in _mac_char_arr
}
