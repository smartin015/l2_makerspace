/* Motion planner for AR3 robotic arm
 *
 * Currently a simple planner that tries to send a periodic stepping
 * signal that matches the position and velocity of state::intent.
 * 
 * `ticks` increments on every call to motion::write(). Whenever
 * it hits ticks_per_step, a step signal is given to the motor.
 *
 * motion::update() frequently updates `ticks_per_step` to meet
 * the trajectory.
 */
#include "log.h"
#include "app_hal.h"
#include "state.h"
#include "motion.h"
#include <cstdio>
#include <cmath> // std::abs
#include <algorithm> // std::min

// Set this wide enough that the step pin interrupt
// can be triggered on the stepper driver - see driver 
// manual for acceptable limits.
// Default: 5 us
#define STEP_PIN_WRITE_USEC 5

// char dbg[2*NUM_J] = "";
bool lim_hit_msg[NUM_J];
float step_vel[NUM_J];
uint32_t ticks_per_step[NUM_J];
uint32_t ticks[NUM_J];
int ticks_since_last_update = 0;

#define VELOCITY_UPDATE_PD_MILLIS 100
#define VELOCITY_MAX_UPDATE_PD_MILLIS 200

#define MAX_ACCEL float(100)
#define MAX_VEL float(10000)
#define MIN_VEL float(10)

#define SIGN(x) ((x > 0) ? 1 : -1)

uint64_t last_velocity_update = 0;
int prev_vel_pos[NUM_J];
float delta_vel[NUM_J];
bool active[NUM_J];

void motion::init() {
  for (int i = 0; i < NUM_J; i++) {
    lim_hit_msg[i] = false;
    step_vel[i] = 0;
    ticks_per_step[i] = 0;
    ticks[i] = 0;
    prev_vel_pos[i] = 0;
    active[i] = false;
    delta_vel[i] = 0;
  }
}

inline void print_state() {
  // LOG_DEBUG("%d %s", int(now), dbg);
  LOG_DEBUG("\tdtick %d\tactive: %d\tpos: want %d got %d\tvel: want %02f got %02f\tdvel %02f --> %lu ticks/step\n", 
      ticks_since_last_update,
      active[0],
      state::intent.pos[0], state::actual.pos[0],
      state::intent.vel[0], state::actual.vel[0],
      delta_vel[0], ticks_per_step[0]);
}

// Velocities are implemented by slowly adjusting 
// the stepping period for each joint based on the target
// velocity and (currently hardcoded) acceleration profile given for each motor
void motion::update() {
  uint64_t now = millis();
  int dt = now - last_velocity_update;
  if (dt < VELOCITY_UPDATE_PD_MILLIS) {
    return;
  }
  last_velocity_update = now;
  if (dt > VELOCITY_MAX_UPDATE_PD_MILLIS) {
    LOG_DEBUG("WARNING: too long between calls to update_velocities(); skipping update\n");
    return; // Avoid edge case in delta processing causing jumps in calculated pos/vel
  }

  for (int i = 0; i < NUM_J; i++) {
    // WARNING: Teensy3.5 has an FPU with hardware support for 32-bit only.
    state::actual.vel[i] = 1000 * float(std::abs(state::actual.pos[i] - prev_vel_pos[i])) / dt;
    prev_vel_pos[i] = state::actual.pos[i];
    
    // Don't calculate stepping if we're already at intent or cannot move
    active[i] = (state::intent.pos[i] - state::actual.pos[i] != 0) && (state::intent.vel[i] != 0);
    if (!active[i]) {
      continue;
    }
    
    if (state::actual.vel[i] == 0) {
      // Shortcut on zero velocity - initial nudge to start moving
      step_vel[i] = MIN_VEL;
    } else {
      // TODO PID loop tuning
      delta_vel[i] = (state::intent.vel[i] - state::actual.vel[i]);

      // Apply acceleration limit
      //if (std::abs(delta_vel[i] * 1000 / dt) > MAX_ACCEL) {
      //  delta_vel[i] = 1000 * MAX_ACCEL * SIGN(delta_vel[i]);
      //}

      // Update velocity, applying firmware velocity limits
      step_vel[i] = std::min(MAX_VEL, std::max(MIN_VEL, step_vel[i] + delta_vel[i]));
    }
  
    // ticks/step = (ticks/sec) * (sec / step)
    //            = (ticks/update * updates/sec) * (1 / step_vel)
    //            = (ticks_since_last_update) * (1000 / dt) * (1 / step_vel)
    ticks_per_step[i] = (1000 * ticks_since_last_update) / (step_vel[i] * dt);
  }

  // print_state();

  ticks_since_last_update = 0;
}

void motion::write() {
  // Continue moving to target
  // Calculate ramp settings

  for (int i = 0; i < NUM_J; i++) {
    if (!active[i]) {
      //dbg[2*i+1] = 'x';
      continue;
    }

    // Don't move if we're driving further into a limit
    int delta = state::intent.pos[i] - state::actual.pos[i];
    uint8_t dir = (delta > 0) ^ ROT_DIR[i];
    //dbg[2*i] = (dir) ? '+' : '-';
    if (!digitalRead(CAL_PIN[i]) && (dir == CAL_DIR[i])) {
      if (!lim_hit_msg[i]) {
        LOG_DEBUG("Driving into limit %d; skipping move\n", i);
        lim_hit_msg[i] = true;
      }
      continue;
    } else {
      lim_hit_msg[i] = false;
    }

    // at least USEC_PER_TICK passes between every counter tick
    ticks[i]++;
    if (ticks[i] < ticks_per_step[i]) {
      continue;
    }

    ticks[i] = 0;
    digitalWrite(DIR_PIN[i], dir);
    digitalWrite(STEP_PIN[i], LOW);
    //dbg[2*i+1] = '.';
  }
  ticks_since_last_update++;

  // Sleep for all steps rather than stepping in sequence 
  // in order to save cycles.
  hal_usleep(STEP_PIN_WRITE_USEC);

  for (int i = 0; i < NUM_J; i++) {
    digitalWrite(STEP_PIN[i], HIGH);
  }
}

void motion::read() {
  for (int i = 0; i < NUM_J; i++) {
    int p = readEnc(i);
    state::actual.pos[i] = p;
  }
}

/*
void set_encoders(const int values[NUM_J]) {
  for (int i = 0; i < NUM_J; i++) {
    writeEnc(i, values[i]);
  }
}
*/
