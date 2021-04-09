#ifndef STATE_H
#define STATE_H

#define VEL_REAL(x) (x >> 8)
#define VEL_FIXED(x) (x << 8)

#include "config.h"
#include <stdint.h>
namespace state {

struct state_t {
  uint8_t mask[NUM_J];
  int16_t pos[NUM_J];
  float vel[NUM_J];
};

extern state_t intent;
extern state_t actual;

} //namespace state

#endif // STATE_H
