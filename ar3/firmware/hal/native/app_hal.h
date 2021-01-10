#ifndef __ARDUINO_SHIM_H__
#define __ARDUINO_SHIM_H__

# define OUTPUT 0
# define INPUT 1
# define INPUT_PULLUP 2
# define HIGH true
#define LOW false

#include <unistd.h>

#define hal_usleep(sec) usleep(sec)

// Swallow pin mode declarations
inline void pinMode(int pin, int mode) {};

void digitalWrite(int pin, bool high);


void initHAL();

#endif // __ARDUINO_SHIM_H__
