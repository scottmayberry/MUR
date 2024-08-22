#ifndef CONFIG_H
#define CONFIG_H

// Define the board type here
// #define BOARD_COMPUTE
#define BOARD_FUSELAGE

// Ensure only one board version is defined
#if defined(BOARD_COMPUTE) && defined(BOARD_FUSELAGE)
  #error "Only one board version should be defined: BOARD_COMPUTE or BOARD_FUSELAGE"
#elif !defined(BOARD_COMPUTE) && !defined(BOARD_FUSELAGE)
  #error "You must define one board version: BOARD_COMPUTE or BOARD_FUSELAGE"
#endif

#define CS_ETH_PIN 10
#define MOSI_pin 11
#define MISO_pin 12
#define SCK_pin 13

#define TRANSMIT_JSON_SIZE 2048
#define RECEIVE_JSON_SIZE 2048

#define LED_PIN 13


#ifdef BOARD_COMPUTE
#define BOARD_ID 0
#elif defined(BOARD_FUSELAGE)
#define BOARD_ID 1
#endif

// UUID configuration
#define UNIQUE_ID_ADDRESS 1
#define UNIQUE_ID_LENGTH 16
#define VALID_FLAG_ADDRESS 0
#define VALID_FLAG_VALUE 0xA5

// // Declare the UUID array as an extern variable
// extern byte uniqueID[UNIQUE_ID_LENGTH];

// // Function prototype to initialize the UUID
// void initializeUUID();
// void printUUID();

#endif // CONFIG_H
