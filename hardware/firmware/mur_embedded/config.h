#ifndef CONFIG_H
#define CONFIG_H

// ========================================
// Board Type Configuration
// ========================================

// Define the board type here by uncommenting the appropriate line.
// Only one board type should be defined at a time to ensure correct pin configurations and functionalities.
// Uncomment `BOARD_COMPUTE` if configuring the compute board.
// Uncomment `BOARD_FUSELAGE` if configuring the fuselage board.

// #define BOARD_COMPUTE
#define BOARD_FUSELAGE

// ========================================
// Board Type Validation
// ========================================

// Ensure that only one board version is defined at a time.
// If both `BOARD_COMPUTE` and `BOARD_FUSELAGE` are defined, throw a compilation error.
// If neither is defined, throw a compilation error.
// This prevents configuration conflicts and ensures the correct board setup.

// Check for conflicting board definitions
#if defined(BOARD_COMPUTE) && defined(BOARD_FUSELAGE)
  #error "Only one board version should be defined: BOARD_COMPUTE or BOARD_FUSELAGE"

// Check if no board is defined
#elif !defined(BOARD_COMPUTE) && !defined(BOARD_FUSELAGE)
  #error "You must define one board version: BOARD_COMPUTE or BOARD_FUSELAGE"
#endif

// ========================================
// SPI Pin Configuration
// ========================================

// Define the SPI (Serial Peripheral Interface) pins used for communication with peripherals like Ethernet controllers.
// These pin assignments are based on the Teensy's hardware SPI pins.
// Ensure that these pins are correctly connected to the corresponding peripherals.

// Chip Select (CS) pin for Ethernet module
#define CS_ETH_PIN 10

// Master Out Slave In (MOSI) pin for SPI communication
#define MOSI_pin 11

// Master In Slave Out (MISO) pin for SPI communication
#define MISO_pin 12

// Serial Clock (SCK) pin for SPI communication
#define SCK_pin 13

// ========================================
// JSON Buffer Sizes
// ========================================

// Define the sizes for JSON buffers used in communication between the robot and external systems.
// These sizes determine how much data can be sent and received in a single JSON message.

// Size of the JSON buffer for transmitting data
#define TRANSMIT_JSON_SIZE 2048

// Size of the JSON buffer for receiving data
#define RECEIVE_JSON_SIZE 2048

// ========================================
// LED Configuration
// ========================================

// Define the pin connected to the LED used for status indication or debugging purposes.
// This LED can be used to provide visual feedback about the robot's state.

// Pin number for the onboard LED
#define LED_PIN 13

// ========================================
// Board Identification
// ========================================

// Assign a unique identifier to each board type for distinguishing between the compute and fuselage boards.
// This ID can be used in communications to identify which board is sending or receiving data.

// Assign BOARD_ID based on the defined board type
#ifdef BOARD_COMPUTE
  #define BOARD_ID 0  // ID for Compute board
#elif defined(BOARD_FUSELAGE)
  #define BOARD_ID 1  // ID for Fuselage board
#endif

// ========================================
// UUID (Universally Unique Identifier) Configuration
// ========================================

// Define memory addresses and parameters related to the UUID used for uniquely identifying each robot.
// This ensures that each MUR robot can be individually recognized and managed.

#define UNIQUE_ID_ADDRESS 1          // Memory address where the unique ID starts
#define UNIQUE_ID_LENGTH 16          // Length of the unique ID in bytes
#define VALID_FLAG_ADDRESS 0         // Memory address where the validity flag is stored
#define VALID_FLAG_VALUE 0xA5        // Value indicating that the unique ID has been successfully initialized and is valid

// ========================================
// UUID Declaration and Initialization (Commented Out)
// ========================================

// The following lines declare the UUID array and function prototypes for initializing and printing the UUID.
// These are currently commented out, possibly because they are defined elsewhere or not needed at the moment.
// Uncomment and implement these lines if UUID functionality is required.

// // Declare the UUID array as an extern variable
// extern byte uniqueID[UNIQUE_ID_LENGTH];

// // Function prototypes to initialize and print the UUID
// void initializeUUID();
// void printUUID();

#endif // CONFIG_H
