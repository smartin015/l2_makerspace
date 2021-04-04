#include "app_hal.h"
#include "state.h"
#include "functions.h"
#include <cstdio>
#include <cmath> // std::abs
#include <algorithm> // std::min

// Set this wide enough that the step pin interrupt
// can be triggered on the stepper driver - see driver 
// manual for acceptable limits.
// Default: 5 us
#define STEPPER_DELAY_USEC 5

bool lim_hit_msg[NUM_J] = {false, false, false, false, false, false};
int step_pd[NUM_J] = {0, 0, 0, 0, 0, 0};
int step_counter[NUM_J] = {0, 0, 0, 0, 0, 0};

#define VELOCITY_UPDATE_PD_MILLIS 100
#define VELOCITY_MAX_DT 200
#define VELMULT 100
#define MAX_STEP_PD_CHANGE 1
#define MAX_STEP_PD (100 * 1000 / STEPPER_DELAY_USEC)
#define MIN_STEP_PD (10 / STEPPER_DELAY_USEC)
uint64_t last_velocity_update = 0;
int prev_vel_pos[NUM_J];
bool step_calc_err[NUM_J];

// Velocities are implemented by slowly adjusting 
// the stepping period for each joint based on the target
// velocity and (currently hardcoded) acceleration profile given for each motor
char dbg[2*NUM_J] = "";
void update_velocities() {
  uint64_t now = millis();
  int dt = now - last_velocity_update;
  if (dt < VELOCITY_UPDATE_PD_MILLIS) {
    return;
  }
  last_velocity_update = now;
  if (dt > VELOCITY_MAX_DT) {
    printf("WARNING: too long between calls to update_velocities(); skipping update\n");
    return; // Avoid edge case in delta processing causing jumps in calculated pos/vel
  }

  for (int i = 0; i < NUM_J; i++) {
    state::actual.vel[i] = std::abs(state::actual.pos[i] - prev_vel_pos[i]) * 100 / dt;
    
    // Don't calculate stepping if we're already at intent
    if (state::intent.pos[i] - state::actual.pos[i] == 0) {
      continue;
    }

    // Intended velocity of zero causes division by zero errors (infinite step pd)
    step_calc_err[i] = (state::intent.vel[i] == 0);
    if (step_calc_err[i]) {
      continue;
    } 
    
    // Shortcut on zero velocity - initial nudge to start moving
    if (state::actual.vel[i] == 0) {
      step_pd[i] = MAX_STEP_PD; 
      continue;
    }

    // Calculate step period based on state::actual.vel rather than current step_pd
    // To account for potential real life variation in stepping speed (e.g. motor slippage)
    int intent_step_pd = (1000000 / state::intent.vel[i]) / STEPPER_DELAY_USEC;
    int actual_step_pd = (1000000 / state::actual.vel[i]) / STEPPER_DELAY_USEC;
    
    // TODO PID loop tuning
    int dpd = intent_step_pd - actual_step_pd;
    if (std::abs(dpd) > MAX_STEP_PD_CHANGE) {
      dpd = MAX_STEP_PD_CHANGE * ((dpd > 0) ? 1 : -1);
    }
    step_pd[i] = std::min(MAX_STEP_PD, std::max(MIN_STEP_PD, step_pd[i] + dpd));
    
    if (i == 0) {
      printf("%d %s", int(now), dbg);
      printf("\tstep_pd %d: ip %d ap %d iv %d av %d istep %d astep %d dpd %d --> %d\n", i, 
          state::intent.pos[0], state::actual.pos[0],
          state::intent.vel[0], state::actual.vel[0],
          intent_step_pd, actual_step_pd, dpd, step_pd[i]);
    }

    prev_vel_pos[i] = state::actual.pos[i];
  }
}

void write_outputs() {
  // Continue moving to target
  // Calculate ramp settings

  for (int i = 0; i < NUM_J; i++) {
    int delta = state::intent.pos[i] - state::actual.pos[i];
    if (delta == 0) {
      // TODO graceful deceleration
      dbg[2*i]='=';
      break;
    }

    // Don't move if we're driving further into a limit
    uint8_t dir = (delta > 0) ^ ROT_DIR[i];
    dbg[2*i] = (dir) ? '+' : '-';
    if (!digitalRead(CAL_PIN[i]) && (dir == CAL_DIR[i])) {
      if (!lim_hit_msg[i]) {
        printf("Driving into limit %d; skipping move\n", i);
        lim_hit_msg[i] = true;
      }
      continue;
    } else {
      lim_hit_msg[i] = false;
    }

    // Larger step_pd == lower frequency stepping.
    // at least STEPPER_DELAY_USEC passes between every counter tick
    step_counter[i]++;
    if (step_calc_err[i]) {
      dbg[2*i+1] = 'e';
      continue;
    }

    if (step_counter[i] < step_pd[i]) {
      continue;
    }

    step_counter[i] = 0;
    digitalWrite(DIR_PIN[i], dir);
    digitalWrite(STEP_PIN[i], LOW);
    dbg[2*i+1] = '.';
  }

  // Sleep for all steps rather than stepping in sequence 
  // in order to save time.
  hal_usleep(STEPPER_DELAY_USEC);
  for (int i = 0; i < NUM_J; i++) {
    digitalWrite(STEP_PIN[i], HIGH);
  }
}

void read_inputs() {
  int values[NUM_J];
  bool hasError = false;
  for (int i = 0; i < NUM_J; i++) {
    int p = readEnc(i);
    state::actual.pos[i] = p;
  }
}

void set_encoders(const int values[NUM_J]) {
  for (int i = 0; i < NUM_J; i++) {
    writeEnc(i, values[i]);
  }
}
