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

#define ABS(v) ((v > 0) ? v : -v)
#define MIN(a,b) ((a<b) ? a : b)
#define MAX(a,b) ((a<b) ? b : a)
#define SIGN(x) ((x > 0) ? 1 : -1)

// Set this wide enough that the step pin interrupt
// can be triggered on the stepper driver - see driver 
// manual for acceptable limits.
// Default: 5 us
#define STEP_PIN_WRITE_USEC 5

// char dbg[2*NUM_J] = "";
bool limit_triggered[NUM_J];
float step_vel[NUM_J];
uint32_t ticks_per_step[NUM_J];
uint32_t ticks[NUM_J];
int ticks_since_last_update = 0;

#define VELOCITY_UPDATE_PD_MILLIS 100
#define VELOCITY_MAX_UPDATE_PD_MILLIS 200

#define MAX_ACCEL float(100)
#define MAX_VEL float(10000)
#define MIN_VEL float(10)


uint64_t last_velocity_update = 0;
int prev_vel_pos[NUM_J];
float delta_vel[NUM_J];
bool active[NUM_J];

void motion::init() {
  for (int i = 0; i < NUM_J; i++) {
    limit_triggered[i] = false;
    step_vel[i] = 0;
    ticks_per_step[i] = 0;
    ticks[i] = 0;
    prev_vel_pos[i] = 0;
    active[i] = false;
    delta_vel[i] = 0;
  }
}

void motion::print_state() {
  // LOG_DEBUG("%d %s", int(now), dbg);
  // NOTE: arduino doesn't include floating point printf by default; scale & cast floats
  LOG_DEBUG("J0 dtick %d on %d\twant m%02x p%d v%d\tgot m%02x p%d v%d --> %d stepvel %lu ticks/step\n", 
      ticks_since_last_update,
      active[0],
      state::intent.mask[0], state::intent.pos[0], int(state::intent.vel[0]*100),
      state::actual.mask[0], state::actual.pos[0], int(state::actual.vel[0]*100),
      int(step_vel[0]*100), ticks_per_step[0]);
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
    LOG_ERROR("too long between calls to update(); skipping");
    ticks_since_last_update = 0;
    return; // Avoid edge case in delta processing causing jumps in calculated pos/vel
  }

  for (int i = 0; i < NUM_J; i++) {
    // WARNING: Teensy3.5 has an FPU with hardware support for 32-bit only.
    state::actual.vel[i] = 1000 * float(ABS(state::actual.pos[i] - prev_vel_pos[i])) / dt;
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
      //if (ABS(delta_vel[i] * 1000 / dt) > MAX_ACCEL) {
      //  delta_vel[i] = 1000 * MAX_ACCEL * SIGN(delta_vel[i]);
      //}

      // Update velocity, applying firmware velocity limits
      step_vel[i] = MIN(MAX_VEL, MAX(MIN_VEL, step_vel[i] + delta_vel[i]));
    }
  
    // ticks/step = (ticks/sec) * (sec / step)
    //            = (ticks/update * updates/sec) * (1 / step_vel)
    //            = (ticks_since_last_update) * (1000 / dt) * (1 / step_vel)
    ticks_per_step[i] = (1000 * ticks_since_last_update) / (step_vel[i] * dt);
  }

  ticks_since_last_update = 0;
}

bool limits_intended = false;
void recalc_limit_intent() {
  // Limits are not at intent if any joint is triggered that does not have
  // the intent mask also set. Otherwise, they are at intent.
  limits_intended = true;
  for (int i = 0; i < NUM_J; i++) {
    if ((state::actual.mask[i] & MASK_LIMIT_TRIGGERED)  && !(state::intent.mask[i] & MASK_LIMIT_TRIGGERED)) {
      limits_intended = false;
    }
  }
}

void motion::intent_changed() {
  recalc_limit_intent();

  for (int i = 0; i < NUM_J; i++) {
    bool en = state::intent.mask[i] & MASK_ENABLED;
    hal::stepEnabled(i, en);
    state::actual.mask[i] = (state::actual.mask[i] & ~MASK_ENABLED) | (en ? MASK_ENABLED : 0);
  }
}

void motion::write() {
  // Continue moving to target
  // Calculate ramp settings
  for (int i = 0; i < NUM_J; i++) {
    if (!active[i] || !limits_intended) {
      //dbg[2*i+1] = 'x';
      continue;
    }

    // Don't move if we're driving further into a limit
    int delta = state::intent.pos[i] - state::actual.pos[i];
    uint8_t dir = (delta > 0);
    //dbg[2*i] = (dir) ? '+' : '-';
    if (!hal::readLimit(i) && !dir) {
      if (!(state::actual.mask[i] & MASK_LIMIT_TRIGGERED)) {
        LOG_DEBUG("Driving into limit %d; skipping move\n", i);
        state::actual.mask[i] |= MASK_LIMIT_TRIGGERED;
      }
      continue;
    } else {
      // Update actual limit state and per-joint limit state
      state::actual.mask[i] &= ~MASK_LIMIT_TRIGGERED;
      recalc_limit_intent();
    }

    // at least USEC_PER_TICK passes between every counter tick
    ticks[i]++;
    if (ticks[i] < ticks_per_step[i]) {
      continue;
    }

    ticks[i] = 0;
    hal::stepDir(i, dir);    
    hal::stepDn(i);
    if (state::intent.mask[i] & MASK_OPEN_LOOP_CONTROL) {
      state::actual.pos[i] += (dir) ? 1 : -1;
    }
    //dbg[2*i+1] = '.';
  }
  ticks_since_last_update++;

  // Sleep for all steps rather than stepping in sequence 
  // in order to save cycles.
  delayMicroseconds(STEP_PIN_WRITE_USEC);

  for (int i = 0; i < NUM_J; i++) {
    hal::stepUp(i);
  }
}

void motion::read() {
  for (int i = 0; i < NUM_J; i++) {
    if (!(state::intent.mask[i] & MASK_OPEN_LOOP_CONTROL)) {
      state::actual.pos[i] = hal::readEnc(i);
    }
  }
}

/*
void set_encoders(const int values[NUM_J]) {
  for (int i = 0; i < NUM_J; i++) {
    hal::writeEnc(i, values[i]);
  }
}
*/
