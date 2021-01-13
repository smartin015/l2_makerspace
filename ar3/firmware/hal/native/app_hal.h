#ifndef __ARDUINO_SHIM_H__
#define __ARDUINO_SHIM_H__

# define OUTPUT 0
# define INPUT 1
# define INPUT_PULLUP 2

#include <unistd.h>
#include "hw.h"

#define hal_usleep(sec) usleep(sec)

inline uint32_t millis() {
  return hw::millis();
}

// Swallow pin mode declarations
inline void pinMode(int pin, int mode) {};
void digitalWrite(int pin, bool high);
bool digitalRead(int pin);
void initHAL();
int readEnc(int idx);
void writeEnc(int idx, int value);

#endif // __ARDUINO_SHIM_H__
