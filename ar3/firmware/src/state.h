#ifndef STATE_H
#define STATE_H

#define VEL_REAL(x) (x >> 8)
#define VEL_FIXED(x) (x << 8)

#include <stdint.h>

// Enable the stepper motor driver for this joint
#define MASK_ENABLED (0b1)

// For intent, this describes whether or not to ignore
// the braking behavior when any limit is triggered.
// For actual, this indicates a limit was triggered.
// Note the arm will stop moving if any joint isn't matching
// the intent.
#define MASK_LIMIT_TRIGGERED (0b1 << 1)

// When this flag is active, do not read encoders for position.
#define MASK_OPEN_LOOP_CONTROL (0b1 << 2)

#define MOTION_MSG_SZ 30
#define SETTINGS_MSG_SZ 14

namespace state {

struct state_t {
  uint8_t mask[NUM_J];
  int16_t pos[NUM_J];
  float vel[NUM_J];
};

struct settings_t {
  float pid[3] = {0.1, 0.1, 0.1}; // Tuning for motion
  int velocity_update_pd_millis = 100;
  float max_accel = 20; // NOTE: must be at least as large as state::settings.initial_spd
  float max_spd = 10000;
  float initial_spd = 10;
};

extern state_t intent;
extern state_t actual;
extern settings_t settings;

void serialize(uint8_t* buf, state_t* state);
void deserialize(state_t* state, uint8_t* buf);
void apply_settings(settings_t* settings, uint8_t* buf);
void print_settings(const settings_t* settings);

} //namespace state

#endif // STATE_H
