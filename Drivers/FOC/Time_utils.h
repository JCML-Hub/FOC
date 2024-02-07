#ifndef TIME_UTILS_H
#define TIME_UTILS_H
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"

void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
uint32_t micros();
uint32_t millis();
#ifdef __cplusplus
}
#endif
#endif