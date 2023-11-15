#ifndef _RATGDO_STATIC_CODE_H
#define _RATGDO_STATIC_CODE_H

// #include <Arduino.h>

#include <stdint.h>
#include <stdbool.h>

void readStaticCode(uint8_t rxSP1StaticCode[SECPLUS1_CODE_LEN], uint8_t *door, uint8_t *light);
void getStaticCode(const char *command);
void printStaticCode(uint8_t code[SECPLUS1_CODE_LEN]);

#endif