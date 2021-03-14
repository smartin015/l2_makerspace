#ifndef STATE_H
#define STATE_H

#include "config.h"
#include <stdint.h>
namespace state {

struct state_t {
  uint8_t mask[NUM_J];
  int16_t pos[NUM_J];
  int16_t vel[NUM_J];
};

extern state_t intent;
extern state_t actual;

} //namespace state

#endif // STATE_H
