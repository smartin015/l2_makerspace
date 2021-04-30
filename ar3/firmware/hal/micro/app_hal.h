#ifndef __ARDUINO_SHIM_H__
#define __ARDUINO_SHIM_H__

#include <Arduino.h>

// NOTE: millis() and delayMicroseconds() already defined

namespace hal {

void init();
void stepDir(int i, bool dir);
void stepDn(int i);
void stepUp(int i);
void stepEnabled(int i, bool en);
bool readLimit(int i);
int readEnc(int idx);
void writeEnc(int idx, int value);

} // namespace hal

#endif // __ARDUINO_SHIM_H__
