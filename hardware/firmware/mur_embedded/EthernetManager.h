#ifndef ETHERNET_MANAGER_H
#define ETHERNET_MANAGER_H

#include "config.h"
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <ArduinoJson.h>


#define BROADCAST_PORT 51584
#define COMM_PORT 51585
#define MILLIS_THRESHOLD_BROADCAST_PING_RATE 10000

#ifdef BOARD_COMPUTE
#define MILLIS_THRESHOLD_BETWEEN_UDP_READS_TO_RESET MILLIS_THRESHOLD_BROADCAST_PING_RATE+1000
#elif defined(BOARD_FUSELAGE)
#define MILLIS_THRESHOLD_BETWEEN_UDP_READS_TO_RESET 300
#endif

class EthernetManager {
public:
  EthernetManager(int csPin);
  bool begin();
  void sendUDPMessageToServer(DynamicJsonDocument& transmitDoc);
  bool processEthernetReceiving(DynamicJsonDocument& receiveDoc);
  bool isServerIPSet();
  void broadcastServerIpRequestIfTimeMet();

private:
  void resetW5500();
  bool initializeEthernet();
  void calculateBroadcastAddress(IPAddress localIP, IPAddress subnetMask, IPAddress &broadcastIP);
  void convertMACToString();
  void timeoutCheckAndUDPReset();

  int _csPin;

  /* https://dnschecker.org/mac-address-generator.php
   NOT ALL MAC ADDRESS ARE VALID! Please use a generator and validate!
   Not all generated mac address work as well. Verify verify verify or connecting to client will fail
   Tested MAC Address:
   {0xB6, 0x11, 0x0E, 0xD1, 0xDE, 0xBD};
   {0x2C, 0x58, 0x87, 0xFB, 0xFB, 0x5C}
   {0xBC, 0xF2, 0x6C, 0x36, 0xDF, 0x8C}
   {0xDC, 0xB0, 0xCA, 0x04, 0xC7, 0x9F}
   {0x50, 0x6C, 0xC1, 0x39, 0xA5, 0x6E}
   {0xE6, 0xF1, 0x57, 0xDF, 0xF6, 0x51}
   {0x92, 0xBA, 0xCF, 0xBD, 0xAE, 0xA5}
   {0xCC, 0x1D, 0x30, 0x1A, 0xDB, 0x71}
   {0x80, 0x6B, 0x09, 0xAF, 0x9C, 0xA8}
   {0xE2, 0x68, 0x1F, 0x02, 0x05, 0x62}
   {0xB8, 0x0B, 0xB3, 0x8D, 0x02, 0xC3}
   */
  #if defined(BOARD_COMPUTE)
  byte _mac[6] = {0xB6, 0x11, 0x0E, 0xD1, 0xDE, 0xBD};
  #elif defined(BOARD_FUSELAGE)
  byte _mac[6] = {0x2C, 0x58, 0x87, 0xFB, 0xFB, 0x5C};
  #endif
  char _mac_char_arr[18];

  elapsedMillis millis_since_last_successful_udp_read;
  elapsedMillis millis_since_last_broadcast_ping;


  EthernetUDP _udp;
  IPAddress _broadcastIP;
  const char _server_for_router_update[20] = "www.example.com";    // Name address for Google (using DNS)
  char packetBuffer[RECEIVE_JSON_SIZE+1];  // buffer to hold incoming packet

  IPAddress _localIP;
  IPAddress _serverIP;
  bool _serverIP_set = false;
  const char _serverIdentifierCharArr[20] = "server\0";  //server identification string
};

#endif // ETHERNET_MANAGER_H