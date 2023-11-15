/************************************
 * Rage
 * Against
 * The
 * Garage
 * Door
 * Opener
 *
 * Copyright (C) 2022  Paul Wieland
 *
 * GNU GENERAL PUBLIC LICENSE
 ************************************/

#ifndef _RATGDO_H
#define _RATGDO_H


/*
temp esp32 pin definitions
*/
// #define D2 14
// #define D7 15
// #define D0 16
// #define D8 17
// #define ASSIGN_OUTPUT_GDO 18


// #include "BootstrapManager.h" // Must use the https://github.com/PaulWieland/arduinoImprovBootstrapper fork, ratgdo branch
// #include "SoftwareSerial.h" // Using espsoftwareserial https://github.com/plerup/espsoftwareserial
#include "rolling_code.h"
#include "static_code.h"
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_attr.h"
// #include "home_assistant.h"

// SoftwareSerial swSerial;

void setup();
void loop();

/********************************** BOOTSTRAP MANAGER *****************************************/
// BootstrapManager bootstrapManager;

/********************************** PIN DEFINITIONS *****************************************/
// MOVE THESE TO kconfig / idf.py menuconfig options!
#define INPUT_GDO  21// 
// #define INPUT_GDO D2 // 
#define OUTPUT_GDO 16 // D1 // D4 // red control terminal / GarageDoorOpener (UART1 TX) pin is D4 on D1 Mini
// #define OUTPUT_GDO ASSIGN_OUTPUT_GDO // D1 // D4 // red control terminal / GarageDoorOpener (UART1 TX) pin is D4 on D1 Mini
// #define TRIGGER_OPEN D5 // dry contact for opening door
// #define TRIGGER_CLOSE D6 // dry contact for closing door
// #define TRIGGER_LIGHT D3 // dry contact for triggering light (no discrete light commands, so toggle only)
// #define STATUS_DOOR D0 // output door status, HIGH for open, LOW for closed
// #define STATUS_OBST D8 // output for obstruction status, HIGH for obstructed, LOW for clear
#define INPUT_OBST 23 // black obstruction sensor terminal
// #define INPUT_OBST D7 // black obstruction sensor terminal


/********************************** MQTT TOPICS & Statuses *****************************************/
// String availabilityStatusTopic = ""; // will be mqttTopicPrefix/deviceName/status/availability
// // online|offline

// String commandTopic = "";     // will be mqttTopicPrefix/deviceName/command/#

// String doorCommandTopic = ""; // will be mqttTopicPrefix/deviceName/command/door
//                               // accepts [open|close|stop]
// String doorStatusTopic = "";  // will be mqttTopicPrefix/deviceName/status/door
uint8_t doorState = 0;
const char* doorStates[7] = {"unknown","open","closed","stopped","opening","closing","syncing"};
typedef enum {
    DOOR_UNKNOWN,
    DOOR_OPEN,
    DOOR_CLOSED,
    DOOR_STOPPED,
    DOOR_OPENING,
    DOOR_CLOSING,
    DOOR_SYNCING
} door_states_t;


// String lightCommandTopic = "";// will be mqttTopicPrefix/deviceName/command/light
//                               // accepts [on|off]
// String lightStatusTopic = ""; // will be mqttTopicPrefix/deviceName/status/light
uint8_t lightState = 2;
const char* lightStates[3] = {"off","on","unknown"};
typedef enum {
    LIGHT_OFF,
    LIGHT_ON,
    LIGHT_UNKNOWN
} light_states_t;


// String lockCommandTopic = ""; // will be mqttTopicPrefix/deviceName/command/lock
//                               // accepts [lock|unlock]
// String lockStatusTopic = "";  // will be mqttTopicPrefix/deviceName/status/lock
uint8_t lockState = 2;
const char* lockStates[3] = {"unlocked","locked","unknown"};
typedef enum {
    LOCK_UNLOCKED,
    LOCK_LOCKED,
    LOCK_UNKNOWN
} lock_states_t;

// String motionStatusTopic = ""; // will be mqttTopicPrefix/deviceName/status/motion
uint8_t motionState = 0;
const char* motionStates[2] = {"clear","detected"};
typedef enum {
    MOTION_CLEAR,
    MOTION_DETECTED
} motion_states_t;

// String obstructionStatusTopic = ""; // will be mqttTopicPrefix/deviceName/status/obstruction
uint8_t obstructionState = 2;
const char* obstructionStates[3] = {"obstructed","clear","unknown"};
typedef enum {
    OBST_OBSTRUCTED,
    OBST_CLEAR,
    OBST_UNKNOWN
} obst_states_t;

/********************************** GLOBAL VARS *****************************************/
bool setupComplete = false;
unsigned int rollingCodeCounter;
unsigned int idCode;
uint8_t txSP1StaticCode[1];
uint8_t rxSP1StaticCode[SECPLUS1_CODE_LEN];
uint8_t secplus1States[19] = {0x35,0x35,0x35,0x35,0x33,0x33,0x53,0x53,0x38,0x3A,0x3A,0x3A,0x39,0x38,0x3A, 0x38,0x3A,0x39,0x3A};

uint8_t txSP2RollingCode[SECPLUS2_CODE_LEN] = {0};
uint8_t rxSP2RollingCode[SECPLUS2_CODE_LEN] = {0};

unsigned int obstructionLowCount = 0;  // count obstruction low pulses
bool obstructionSensorDetected = false;
unsigned long lastObstructionHigh = 0;  // count time between high pulses from the obst ISR
unsigned long lastRX = 0;

bool dryContactDoorOpen = false;
bool dryContactDoorClose = false;
bool dryContactToggleLight = false;

/********************************** FUNCTION DECLARATION *****************************************/
// void callback(char *topic, uint8_t *payload, unsigned int length);
// void manageDisconnections();
// void manageQueueSubscription();
// void manageHardwareButton();

void blink(bool trigger);
void transmit(uint8_t* payload, unsigned int length);
void sync();

void toggleDoor();
void openDoor();
void closeDoor();
void stopDoor();
void sendDoorStatus();

void toggleLight();
void lightOn();
void lightOff();
void sendLightStatus();

void toggleLock();
void lock();
void unlock();
void sendLockStatus();

void sendMotionStatus();

void obstructionLoop();
void sendObstructionStatus();

void statusUpdateLoop();

void gdoStateLoop();
// void dryContactLoop();
// void wallPanelEmulatorLoop();

void pullLow();

/********************************** INTERRUPT SERVICE ROUTINES ***********************************/
// void IRAM_ATTR isrDebounce(const char *type);
// void IRAM_ATTR isrDoorOpen();
// void IRAM_ATTR isrDoorClose();
// void IRAM_ATTR isrLight();
void IRAM_ATTR isrObstruction();
// void IRAM_ATTR isrRPM1();
// void IRAM_ATTR isrRPM2();

#endif