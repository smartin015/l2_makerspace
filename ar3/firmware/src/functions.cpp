#include "app_hal.h"
#include "state.h"
#include "functions.h"
#include <cstdio>

// Set this wide enough that the step pin interrupt
// can be triggered on the stepper driver - see driver 
// manual for acceptable limits.
#define STEPPER_DELAY_USEC 5

bool lim_hit_msg[NUM_J] = {false, false, false, false, false, false};
int step_pd[NUM_J] = {0, 0, 0, 0, 0, 0};
int step_counter[NUM_J] = {0, 0, 0, 0, 0, 0};

#define VELOCITY_UPDATE_PD_MILLIS 100
#define MAX_STEP_PD_CHANGE 1
uint64_t last_velocity_update = 0;
int prev_vel_pos[NUM_J];
bool step_calc_err[NUM_J];

// Velocities are implemented by slowly adjusting 
// the stepping period for each joint based on the target
// velocity and (currently hardcoded) acceleration profile given for each motor
void update_velocities() {
  uint64_t now = millis();
  int dt = now - last_velocity_update;
  if (dt < VELOCITY_UPDATE_PD_MILLIS) {
    return;
  }
  last_velocity_update = now;

  for (int i = 0; i < NUM_J; i++) {
    state::actual.vel[i] = (state::actual.pos[i] - prev_vel_pos[i]) / dt;
    
    // Don't calculate stepping if we're already at intent
    if (state::intent.pos[i] - state::actual.pos[i] == 0) {
      continue;
    }

    // Intended velocity of zero causes division by zero errors (infinite step pd)
    step_calc_err[i] = (state::intent.vel[i] == 0);
    if (step_calc_err[i]) {
      continue;
    } 
    
    // Calculate step period based on state::actual.vel rather than current step_pd
    // To account for potential real life variation in stepping speed (e.g. motor slippage)
    int intent_step_pd = (1000000 / state::intent.vel[i]) / STEPPER_DELAY_USEC;
    int actual_step_pd = (1000000 / state::actual.vel[i]) / STEPPER_DELAY_USEC;
    int dpd = (intent_step_pd - actual_step_pd);
        
    // TODO PID loop tuning
    if (((dpd < 0) ? -dpd : dpd) > MAX_STEP_PD_CHANGE) {
      dpd = MAX_STEP_PD_CHANGE * ((dpd > 0) ? 1 : -1);
    }
    step_pd[i] += dpd;
    if (i == 0) {
      printf("Step_pd %d update, +%d --> %d", i, dpd, step_pd[i]);
    }
  }
}

void write_outputs() {
  // Continue moving to target
  // Calculate ramp settings
  uint64_t now = millis();  // todo rm
  char dbg[2*NUM_J];

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
  printf("%d %s\n", now, dbg);
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
