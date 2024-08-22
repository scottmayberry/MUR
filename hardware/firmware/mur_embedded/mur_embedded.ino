#include "config.h"
#include "EthernetManager.h"
#include "SensorManager.h"
#include "ThrusterManager.h"

// StaticJsonDocument<RECEIVE_JSON_SIZE> receiveDoc;

EthernetManager ethernetManager(CS_ETH_PIN);
SensorManager sensorManager;

#ifdef BOARD_FUSELAGE
ThrusterManager thrusterManager;
#endif


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


void setup() {
  Serial.begin(115200);
  SPI.begin();
  delay(500);
//   print_json_capacity();
  delay(100);
  startEthernet();
  delay(100);

  sensorManager.setupSensors();
  delay(100);

  #ifdef BOARD_FUSELAGE
  thrusterManager.setupThrusters();
  #endif

  Serial.println("Starting MUR");
  delay(100);
}

void startEthernet(){
   while (!ethernetManager.begin()) {
    Serial.println("Failed to initialize Ethernet. Retrying.");
    delay(1000);
  }
}

void loop() {
    delay(1);

    //clear the JsonDocuments to retrieve memory
    bool ready_to_transmit = false;
    bool received_message = false;

    //set the documents
    DynamicJsonDocument transmitDoc(TRANSMIT_JSON_SIZE);
    DynamicJsonDocument receiveDoc(RECEIVE_JSON_SIZE);
    // print_json_capacity(transmitDoc, receiveDoc);
    
    
    if (ethernetManager.isServerIPSet())
    {
        //receive messages and handle incoming commands
        received_message = ethernetManager.processEthernetReceiving(receiveDoc);
        #ifdef BOARD_FUSELAGE
        thrusterManager.processThrusters(received_message, receiveDoc, transmitDoc);
        #endif

        //collect data and transmit
        ready_to_transmit = sensorManager.processSensors(transmitDoc); //add sensor data to transmitDoc
        if(ready_to_transmit)
        {
            ethernetManager.sendUDPMessageToServer(transmitDoc);
        }
    } else {
        ethernetManager.processEthernetReceiving(receiveDoc);
        // if ethernetManager gets a server, reset sensorManager timers
        if (ethernetManager.isServerIPSet()){
            sensorManager.reset_timers();
        }
    }

    // Broadcast Server IP on broadcastIP to get the Server IP (server sends server message)
    ethernetManager.broadcastServerIpRequestIfTimeMet();

    //reset docs
    transmitDoc.clear();
    receiveDoc.clear();
}