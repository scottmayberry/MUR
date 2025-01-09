// EthernetManager.h
// Header file for the EthernetManager class.
// Manages Ethernet connectivity, UDP communication, and network configurations
// for the compute and battery fuselage boards of the MUR.

#ifndef ETHERNET_MANAGER_H
#define ETHERNET_MANAGER_H

#include "config.h"            // Include configuration settings
#include <SPI.h>               // Include SPI library for communication with Ethernet controller
#include <Ethernet.h>          // Include Ethernet library for network functionalities
#include <EthernetUdp.h>       // Include Ethernet UDP library for UDP communication
#include <ArduinoJson.h>       // Include ArduinoJson library for JSON handling

// Define constants for broadcast and communication ports
#define BROADCAST_PORT 51584
#define COMM_PORT 51585

// Define the threshold for broadcast ping rate in milliseconds
#define MILLIS_THRESHOLD_BROADCAST_PING_RATE 10000  // 10 seconds

// Conditional compilation based on the board type to set UDP reset thresholds
#ifdef BOARD_COMPUTE
  #define MILLIS_THRESHOLD_BETWEEN_UDP_READS_TO_RESET MILLIS_THRESHOLD_BROADCAST_PING_RATE+1000  // 11 seconds for compute board
#elif defined(BOARD_FUSELAGE)
  #define MILLIS_THRESHOLD_BETWEEN_UDP_READS_TO_RESET 300  // 300 milliseconds for fuselage board
#endif

// EthernetManager class declaration
class EthernetManager {
public:
  // Constructor: Initializes the EthernetManager with the specified chip select pin
  EthernetManager(int csPin);
  
  // Initializes Ethernet and network configurations
  bool begin();
  
  // Sends a UDP message to the server with the provided JSON document
  void sendUDPMessageToServer(DynamicJsonDocument& transmitDoc);
  
  // Processes incoming Ethernet data and deserializes JSON if applicable
  bool processEthernetReceiving(DynamicJsonDocument& receiveDoc);
  
  // Checks if the server IP has been set
  bool isServerIPSet();
  
  // Broadcasts a server IP request if the specified time threshold has been met
  void broadcastServerIpRequestIfTimeMet();

private:
  // Resets the W5500 Ethernet controller via SPI communication
  void resetW5500();
  
  // Initializes Ethernet using DHCP, retrying up to 5 times if necessary
  bool initializeEthernet();
  
  // Calculates the broadcast IP address based on the local IP and subnet mask
  void calculateBroadcastAddress(IPAddress localIP, IPAddress subnetMask, IPAddress &broadcastIP);
  
  // Converts the MAC address byte array to a human-readable string format
  void convertMACToString();
  
  // Checks for UDP read timeouts and resets the UDP connection if the threshold is exceeded
  void timeoutCheckAndUDPReset();

  int _csPin;  // Chip Select (CS) pin for SPI communication with the Ethernet controller

  /*
   * MAC Address Configuration:
   * Ensure that all MAC addresses are valid and unique.
   * Invalid or duplicate MAC addresses can cause network conflicts or connection failures.
   * Refer to a reliable MAC address generator and validate the generated addresses.
   *
   * Example MAC address generator: https://dnschecker.org/mac-address-generator.php
   * 
   * Example Valid MAC Addresses:
   * {0xB6, 0x11, 0x0E, 0xD1, 0xDE, 0xBD};
   * {0x2C, 0x58, 0x87, 0xFB, 0xFB, 0x5C};
   * {0xBC, 0xF2, 0x6C, 0x36, 0xDF, 0x8C};
   * {0xDC, 0xB0, 0xCA, 0x04, 0xC7, 0x9F};
   * {0x50, 0x6C, 0xC1, 0x39, 0xA5, 0x6E};
   * {0xE6, 0xF1, 0x57, 0xDF, 0xF6, 0x51};
   * {0x92, 0xBA, 0xCF, 0xBD, 0xAE, 0xA5};
   * {0xCC, 0x1D, 0x30, 0x1A, 0xDB, 0x71};
   * {0x80, 0x6B, 0x09, 0xAF, 0x9C, 0xA8};
   * {0xE2, 0x68, 0x1F, 0x02, 0x05, 0x62};
   * {0xB8, 0x0B, 0xB3, 0x8D, 0x02, 0xC3};
   */
  
  // Define the MAC address based on the board type
  #if defined(BOARD_COMPUTE)
    byte _mac[6] = {0xB6, 0x11, 0x0E, 0xD1, 0xDE, 0xBD};  // MAC address for compute board
  #elif defined(BOARD_FUSELAGE)
    byte _mac[6] = {0x2C, 0x58, 0x87, 0xFB, 0xFB, 0x5C};  // MAC address for fuselage board
  #endif
  
  char _mac_char_arr[18];  // Array to store the MAC address as a string (e.g., "XX:XX:XX:XX:XX:XX")

  elapsedMillis millis_since_last_successful_udp_read;  // Timer for tracking time since last successful UDP read
  elapsedMillis millis_since_last_broadcast_ping;        // Timer for tracking time since last broadcast ping

  EthernetUDP _udp;               // EthernetUDP instance for handling UDP communication
  IPAddress _broadcastIP;         // Calculated broadcast IP address based on local IP and subnet mask
  const char _server_for_router_update[20] = "www.example.com";  // Server address for router updates (using DNS)
  char packetBuffer[RECEIVE_JSON_SIZE+1];  // Buffer to hold incoming UDP packets (+1 for null-termination)

  IPAddress _localIP;             // Local IP address assigned via DHCP
  IPAddress _serverIP;            // Server IP address obtained from incoming packets
  bool _serverIP_set = false;     // Flag indicating whether the server IP has been set
  const char _serverIdentifierCharArr[20] = "server\0";  // Identifier string to recognize server messages
};

#endif // ETHERNET_MANAGER_H
