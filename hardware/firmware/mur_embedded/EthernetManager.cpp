#include "config.h"
#include "EthernetManager.h"

EthernetManager::EthernetManager(int csPin)
  : _csPin(csPin) {}

bool EthernetManager::begin() {
  Ethernet.init(_csPin);
  SPI.begin();

  convertMACToString();
  Serial.print("MAC: ");
  Serial.println(_mac_char_arr);
  
  resetW5500();

  if (!initializeEthernet()) {
    Serial.println("Failed to initialize Ethernet");
    return false;
  }

  IPAddress subnetMask = Ethernet.subnetMask();
  _localIP = Ethernet.localIP();
  calculateBroadcastAddress(_localIP, subnetMask, _broadcastIP);
  Serial.print("Broadcast IP: \t");
  Serial.println(_broadcastIP);

  Serial.print("My IP address: \t");
  Serial.println(Ethernet.localIP());

  Serial.print("Subnet Mask: \t");
  Serial.println(Ethernet.subnetMask());

  Serial.print("Gateway IP: \t");
  Serial.println(Ethernet.gatewayIP());

  Serial.print("DNS Server IP: \t");
  Serial.println(Ethernet.dnsServerIP());

  Serial.print("Connecting to ");
  Serial.print(_server_for_router_update);
  Serial.println("...");

  EthernetClient client;

  for(int retry = 0; retry < 10; retry++){
    // If you get a connection, report back via serial:
    if (client.connect(_server_for_router_update, 80)) {
      Serial.print("Connected to ");
      Serial.println(client.remoteIP());
      // Make an HTTP request:
      client.println("GET /search?q=arduino HTTP/1.1");
      client.println("Host: www.example.com");
      client.println("Connection: close");
      client.println();
      break;
    } else {
      // If you didn't get a connection to the server:
      Serial.println("Connection failed");
      delay(1000);
    }
  }
  _udp.begin(COMM_PORT);
  millis_since_last_successful_udp_read = 0;
  return true;
}

void EthernetManager::sendUDPMessageToServer(DynamicJsonDocument& transmitDoc) {
  transmitDoc["id"] = BOARD_ID;
  _udp.beginPacket(_serverIP, COMM_PORT);
  serializeJson(transmitDoc, _udp);
  _udp.endPacket();
  delay(1);
}

void EthernetManager::broadcastServerIpRequestIfTimeMet() {
  if(millis_since_last_broadcast_ping < MILLIS_THRESHOLD_BROADCAST_PING_RATE){
    return;
  }
  millis_since_last_broadcast_ping = 0;
  DynamicJsonDocument transmitDoc(TRANSMIT_JSON_SIZE);
  transmitDoc["id"] = BOARD_ID;
  transmitDoc["serverRequestForIP"] = _localIP;
  transmitDoc["mac"] = _mac_char_arr;
  transmitDoc["BROADCAST_PORT"] = BROADCAST_PORT;
  transmitDoc["COMM_PORT"] = COMM_PORT;
  if(isServerIPSet()){
    transmitDoc["serverIP"] = _serverIP;
  }

  _udp.beginPacket(_broadcastIP, BROADCAST_PORT);
  serializeJson(transmitDoc, _udp);
  _udp.endPacket();
  delay(1);
}

bool EthernetManager::isServerIPSet(){
  return _serverIP_set;
}

bool EthernetManager::processEthernetReceiving(DynamicJsonDocument& receiveDoc) {
  int packetSize = _udp.parsePacket();
  if (packetSize) {
    millis_since_last_successful_udp_read = 0;
    // Check if the packet size exceeds the buffer size
    if (packetSize >= RECEIVE_JSON_SIZE) {
      Serial.print("ERROR PACKET TOO LARGE. DISCARDING...");
      // Read the packet directly to the serial port
      char discardBuffer[packetSize];
      _udp.read(discardBuffer, packetSize);
      Serial.println("...Discard complete");
      delay(1);
      return false;
    }

    _udp.read(packetBuffer, RECEIVE_JSON_SIZE);
    packetBuffer[packetSize] = '\0';

    Serial.println(packetBuffer);

    if (strcmp(packetBuffer, _serverIdentifierCharArr) == 0){
        _serverIP = _udp.remoteIP();
        _serverIP_set = true;
        Serial.print("Server IP is ");
        Serial.println(_serverIP);
        delay(1);
        return false;
    }
    
    // Check up to the first 8 bytes for '{' or '['
    int startIdx = -1;
    for (int i = 0; i < 8 && i < packetSize; i++) {
      if (packetBuffer[i] == '{' || packetBuffer[i] == '[') {
        startIdx = i;
        break;
      }
    }

    // If neither '{' nor '[' was found in the first 8 bytes, return false
    if (startIdx == -1) {
      delay(1);
      return false;
    }

    receiveDoc.clear();
    DeserializationError error = deserializeJson(receiveDoc, packetBuffer + startIdx);
    if (error) {
      receiveDoc.clear();
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      delay(1);
      return false;
    }
    // Process _receiveDoc as needed
    delay(1);
    return true;
  } else{
    // if you dont receive a broadcast, reset the udp connection
    timeoutCheckAndUDPReset();
  }
  delay(1);
  return false;
}

void EthernetManager::timeoutCheckAndUDPReset() {
  if(millis_since_last_successful_udp_read >= MILLIS_THRESHOLD_BETWEEN_UDP_READS_TO_RESET){
    millis_since_last_successful_udp_read = 0;
    Serial.print("Resetting UDP: ");
    Serial.println(millis());
    _udp.stop();
    delay(2);
    _udp.begin(COMM_PORT);
    delay(2);
  }
}

void EthernetManager::resetW5500() {
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(0x00);
  SPI.transfer(0x80);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  delay(200);
}

bool EthernetManager::initializeEthernet() {
  Serial.println("Initialize Ethernet with DHCP:");
  for (int retry = 0; retry < 5; retry++) {
    if (Ethernet.begin(_mac) != 0) {
      Serial.print("  DHCP assigned IP ");
      Serial.println(Ethernet.localIP());
      return true;
    } else {
      Serial.println("Failed to configure Ethernet using DHCP. Retrying...");
      if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found. Sorry, can't run without hardware. :(");
        return false;
      }
      if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected.");
      }
      resetW5500();
      digitalWrite(LED_PIN, HIGH);
      delay(1000);
      digitalWrite(LED_PIN, LOW);
    }
  }
  return false;
}

void EthernetManager::calculateBroadcastAddress(IPAddress localIP, IPAddress subnetMask, IPAddress &broadcastIP) {
  for (int i = 0; i < 4; i++) {
    broadcastIP[i] = localIP[i] | ~subnetMask[i];
  }
}

void EthernetManager::convertMACToString() {
    snprintf(_mac_char_arr, sizeof(_mac_char_arr), "%02X:%02X:%02X:%02X:%02X:%02X", 
             _mac[0], _mac[1], _mac[2], _mac[3], _mac[4], _mac[5]);
}
