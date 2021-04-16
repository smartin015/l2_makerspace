#ifndef __ARDUINO_SHIM_H__
#define __ARDUINO_SHIM_H__

#include <Arduino.h>

#define hal_usleep(sec) delayMicroseconds(sec)

int readEnc(int idx);
void writeEnc(int idx, int value);

#endif // __ARDUINO_SHIM_H__
