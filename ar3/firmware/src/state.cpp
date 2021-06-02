#include "state.h"
#include "log.h"

inline void int16buf(uint8_t* buf, int16_t i) {
  buf[0] = i & 0xff;
  buf[1] = (i >> 8) & 0xff;
}

inline void int32buf(uint8_t* buf, int32_t i) {
  buf[0] = i & 0xff;
  buf[1] = (i >> 8) & 0xff;
  buf[2] = (i >> 16) & 0xff;
  buf[3] = (i >> 24) & 0xff;
}

inline int16_t buf2int16(uint8_t* buf) {
  return buf[0] + (buf[1] << 8);
}

inline int32_t buf2int32(uint8_t* buf) {
  return buf[0] + (buf[1] << 8) + (buf[2] << 16) + (buf[3] << 24);
}

namespace state {
  void serialize(uint8_t* buf, state_t* state) {
    for (int i = 0; i < NUM_J; i++) {
      buf[i] = state->mask[i];
      int32buf(buf + NUM_J + sizeof(int32_t)*i, state->pos[i]);
      int16buf(buf + NUM_J + sizeof(int32_t)*NUM_J + sizeof(int16_t)*i, int16_t(state->vel[i]));
    }
  }

  void deserialize(state_t* state, uint8_t* buf) {
    for (int i = 0; i < NUM_J; i++) {
      state->mask[i] = buf[i];
      state->pos[i] = buf2int32(buf + NUM_J + sizeof(int32_t)*i);
      state->vel[i] = float(buf2int16(buf + NUM_J + sizeof(int32_t)*NUM_J + sizeof(int16_t)*i));
      // printf("J%d %d %x %d %x -> %d\n", i, NUM_J + 2*i, buf[NUM_J + 2*i], NUM_J + 2*i+1, buf[NUM_J + 2*i+1], state->pos[i]);
    }
  }

  void apply_settings(settings_t* s, uint8_t* buf) {
    s->pid[0] = float(buf2int16(buf)) / 1000; // Tuning for motion
    s->pid[1] = float(buf2int16(buf+2)) / 1000;
    s->pid[2] = float(buf2int16(buf+4)) / 1000;
    s->velocity_update_pd_millis = buf2int16(buf+6);
    s->max_accel = float(buf2int16(buf+8)) / 1000;
    s->max_spd = float(buf2int16(buf+10));
    s->initial_spd = float(buf2int16(buf+12)) / 1000;
  }

  void print_settings(const settings_t* s) {
    LOG_INFO("VEL_UP_MS%d (P%d I%d D%d MAX_A%d MAX_S%d INIT_S%d)", 
        s->velocity_update_pd_millis,  
        int(s->pid[0]*100), 
        int(s->pid[1]*100), 
        int(s->pid[2]*100), 
        int(s->max_accel*100),
        int(s->max_spd*100),
        int(s->initial_spd*100));
  }

  state_t intent;
  state_t actual;
  settings_t settings;

} // namespace state
