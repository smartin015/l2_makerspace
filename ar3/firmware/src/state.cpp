#include "state.h"

namespace state {

  void serialize(uint8_t* buf, state_t* state) {
    for (int i = 0; i < NUM_J; i++) {
      buf[i] = state->mask[i];
      buf[NUM_J + 2*i] = state->pos[i] & 0xff;
      buf[NUM_J + 2*i + 1] = (state->pos[i] >> 8) & 0xff;
      buf[3*NUM_J + 2*i] = int16_t(state->vel[i]) & 0xff;
      buf[3*NUM_J + 2*i + 1] = (int16_t(state->vel[i]) >> 8) & 0xff;
    }
  }

  void deserialize(state_t* state, uint8_t* buf) {
    for (int i = 0; i < NUM_J; i++) {
      state->mask[i] = buf[i];
      state->pos[i] = buf[NUM_J + 2*i] + (buf[NUM_J + 2*i + 1] << 8);
      state->vel[i] = float(buf[3*NUM_J + 2*i] + (buf[3*NUM_J + 2*i + 1] << 8));
      // printf("J%d %d %x %d %x -> %d\n", i, NUM_J + 2*i, buf[NUM_J + 2*i], NUM_J + 2*i+1, buf[NUM_J + 2*i+1], state->pos[i]);
    }
  }

  state_t intent;
  state_t actual;

} // namespace state
