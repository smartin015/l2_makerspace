#ifndef __ARDUINO_SHIM_H__
#define __ARDUINO_SHIM_H__

#include <unistd.h>
#include "hw.h"

// No need to delay in native environment (non-realtime OS, makes us miss our tick freq deadline)
#define delayMicroseconds(sec)

inline uint32_t millis() {
  return hw::millis();
}

namespace hal {

  void init();
  void stepDir(int i, bool dir);
  void stepDn(int i);
  void stepUp(int i);
  void stepEnabled(int i, bool en);
  bool readLimit(int i);
  int readEnc(int idx);
  void writeEnc(int idx, int value);
  void startMainTimer(int hz, void(*cb)());
  void disableInterrupts();
  void enableInterrupts();

} //namespace hal

#endif // __ARDUINO_SHIM_H__
