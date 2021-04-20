#ifndef STATE_H
#define STATE_H

#define VEL_REAL(x) (x >> 8)
#define VEL_FIXED(x) (x << 8)

#include <stdint.h>

// For intent, this describes whether or not to ignore
// the braking behavior when any limit is triggered.
// For actual, this indicates a limit was triggered.
// Note the arm will stop moving if any joint isn't matching
// the intent.
#define MASK_LIMIT_TRIGGERED 0b00000001

namespace state {

struct state_t {
  uint8_t mask[NUM_J];
  int16_t pos[NUM_J];
  float vel[NUM_J];
};

extern state_t intent;
extern state_t actual;

void serialize(uint8_t* buf, state_t* state);
void deserialize(state_t* state, uint8_t* buf);

} //namespace state

#endif // STATE_H
