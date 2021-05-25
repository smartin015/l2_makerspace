#include "state.h"
#include "log.h"

inline void int2buf(uint8_t* buf, int16_t i) {
  buf[0] = i & 0xff;
  buf[1] = (i >> 8) & 0xff;
}

inline int16_t buf2int(uint8_t* buf) {
  return buf[0] + (buf[1] << 8);
}

namespace state {
  void serialize(uint8_t* buf, state_t* state) {
    for (int i = 0; i < NUM_J; i++) {
      buf[i] = state->mask[i];
      int2buf(buf + NUM_J + 2*i, state->pos[i]);
      int2buf(buf + 3*NUM_J + 2*i, int16_t(state->vel[i]));
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
    s->pid[0] = float(buf2int(buf)) / 1000; // Tuning for motion
    s->pid[1] = float(buf2int(buf+2)) / 1000;
    s->pid[2] = float(buf2int(buf+4)) / 1000;
    s->velocity_update_pd_millis = buf2int(buf+6);
    s->max_accel = float(buf2int(buf+8)) / 1000;
    s->max_spd = float(buf2int(buf+10));
    s->initial_spd = float(buf2int(buf+12)) / 1000;
  }

  void print_settings(const settings_t* s) {
    LOG_INFO("VEL_UP_MS%d (P%f I%f D%f MAX_A%f MAX_S%f INIT_S%f)", 
        s->velocity_update_pd_millis,  
        s->pid[0], 
        s->pid[1], 
        s->pid[2], 
        s->max_accel,
        s->max_spd,
        s->initial_spd);
  }

  state_t intent;
  state_t actual;
  settings_t settings;

} // namespace state
