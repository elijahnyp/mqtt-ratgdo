#ifndef _RATGDO_ROLLING_CODE_H
#define _RATGDO_ROLLING_CODE_H

// #include <Arduino.h>
// #include <LittleFS.h>
// #include <ArduinoJson.h>
// #include "BootstrapManager.h"

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "secplus.h"
#ifdef __cplusplus
}
#endif

esp_err_t readCounterFromFlash(const char *type, unsigned int *counter); // get the rolling code counter from setup.json & return it
esp_err_t writeCounterToFlash(const char *type, unsigned int *counter); // write the counter back to setup.json
void readRollingCode(uint8_t rxSP2RollingCode[SECPLUS2_CODE_LEN], uint8_t *door, uint8_t *light, uint8_t *lock, uint8_t *motion, uint8_t *obstruction);
void getRollingCode(const char *command); // get the next rolling code for type [reboot1,reboot2,reboot3,reboot4,reboot5,door1,light]
void printRollingCode(uint8_t code[SECPLUS2_CODE_LEN]);
esp_err_t deleteCounter(const char* type);

#endif