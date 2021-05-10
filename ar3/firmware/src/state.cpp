#include "state.h"

inline int16_t buf2int(uint8_t* buf) {
  return buf[0] + (buf[1] << 8);
}

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
      state->pos[i] = buf2int(buf + NUM_J + 2*i);
      state->vel[i] = float(buf2int(buf + 3*NUM_J + 2*i));
      // printf("J%d %d %x %d %x -> %d\n", i, NUM_J + 2*i, buf[NUM_J + 2*i], NUM_J + 2*i+1, buf[NUM_J + 2*i+1], state->pos[i]);
    }
  }

  void apply_settings(settings_t* s, uint8_t* buf) {
    s->pid[0] = buf2int(buf); // Tuning for motion
    s->pid[1] = buf2int(buf+2);
    s->pid[2] = buf2int(buf+4);
    s->velocity_update_pd_millis = buf2int(buf+6);
    s->max_accel = buf2int(buf+8);
    s->max_spd = buf2int(buf+10);
    s->initial_spd = buf2int(buf+12);
  }

  state_t intent;
  state_t actual;
  settings_t settings;

} // namespace state
