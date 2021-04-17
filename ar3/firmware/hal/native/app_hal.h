#ifndef __ARDUINO_SHIM_H__
#define __ARDUINO_SHIM_H__

#include <unistd.h>
#include "hw.h"

#define delayMicroseconds(sec) usleep(sec)

inline uint32_t millis() {
  return hw::millis();
}

namespace hal {

void initJoint(int i);
void stepDir(int i, bool dir);
void stepDn(int i);
void stepUp(int i);
bool readLimit(int i);
int readEnc(int i);
void writeEnc(int i, int value);

} //namespace hal

#endif // __ARDUINO_SHIM_H__
