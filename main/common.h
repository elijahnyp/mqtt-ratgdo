// #include <Arduino.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define SECPLUS2_CODE_LEN 19 // the length of each command sent to the door.
extern uint8_t txSP2RollingCode[SECPLUS2_CODE_LEN];
extern uint8_t rxSP2RollingCode[SECPLUS2_CODE_LEN];
extern unsigned int rollingCodeCounter;
extern unsigned int idCode;

#define SECPLUS1_CODE_LEN 2
extern uint8_t txSP1StaticCode[1];
extern uint8_t rxSP1StaticCode[SECPLUS1_CODE_LEN];