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
uint32_t ticks_since_last_print = 0;
uint32_t steps_since_last_print = 0; // Cumulative, all joints

#define VELOCITY_UPDATE_PD_MILLIS 100
#define VELOCITY_MAX_UPDATE_PD_MILLIS 200

#define MAX_ACCEL float(20) // NOTE: must be at least as large as MIN_SPD
#define MAX_SPD float(10000)
#define MIN_SPD float(10)

const float PID[3] = {0.1, 0.1, 0.1};

uint64_t last_velocity_update = 0;
float prev_vel[NUM_J];
float prev_err_vel[NUM_J];
int prev_pos[NUM_J];
float err_vel[NUM_J];
int err_pos[NUM_J];
bool active[NUM_J];

void motion::init() {
  for (int i = 0; i < NUM_J; i++) {
    limit_triggered[i] = false;
    step_vel[i] = 0;
    ticks_per_step[i] = 0;
    ticks[i] = 0;
    prev_pos[i] = 0;
    prev_vel[i] = 0;
    prev_err_vel[i] = 0;
    active[i] = false;
    err_vel[i] = 0;
    err_pos[i] = 0;
  }
}

void motion::print_state() {
  // LOG_DEBUG("%d %s", int(now), dbg);
  // NOTE: arduino doesn't include floating point printf by default; scale & cast floats
  LOG_DEBUG("J0 dtick %lu\tdstep %lu\ton %d\twant m%02x p%d v%d\tgot m%02x p%d v%d --> stepvel %d ticks/step %lu\n", 
      ticks_since_last_print,
      steps_since_last_print,
      active[0],
      state::intent.mask[0], state::intent.pos[0], int(state::intent.vel[0])*100,
      state::actual.mask[0], state::actual.pos[0], int(state::actual.vel[0])*100,
      int(step_vel[0])*100, ticks_per_step[0]);
  ticks_since_last_print = 0;
  steps_since_last_print = 0;
}

// Velocities are implemented by slowly adjusting 
// the stepping period for each joint based on the target
// velocity and (currently hardcoded) acceleration profile given for each motor
bool motion::update() {
  uint64_t now = millis();
  int dt = now - last_velocity_update;
  if (dt < VELOCITY_UPDATE_PD_MILLIS) {
    return false;
  }
  last_velocity_update = now;
  if (dt > VELOCITY_MAX_UPDATE_PD_MILLIS) {
    LOG_ERROR("too long between calls to update(); skipping");
    return false; // Avoid edge case in delta processing causing jumps in calculated pos/vel
  }

  // Prevent motor steps while we recalculate ticks per step
  hal::disableInterrupts();
   
  // WARNING: Teensy3.5 has an FPU with hardware support for 32-bit only.
  for (int i = 0; i < NUM_J; i++) {
    // Update kinematic state
    state::actual.vel[i] = (1000 * float(state::actual.pos[i] - prev_pos[i])) / dt;
    prev_pos[i] = state::actual.pos[i];
    prev_vel[i] = state::actual.vel[i];
    
    prev_err_vel[i] = err_vel[i];
    err_vel[i] = state::intent.vel[i] - state::actual.vel[i];
    err_pos[i] = state::intent.pos[i] - state::actual.pos[i];

    // Don't calculate stepping if we're already at intent
    active[i] = (err_vel[i] != 0) || (err_pos[i] != 0);
    if (!active[i]) {
      continue;
    }
    
    // This is PID adjustment targeting velocity - in this case, P=velocity, I=position, D=acceleration
    // Note that we have targets both for position and velocity, but not for acceleration - that's where
    // we use a real value.
    float vel_adjust = (PID[0] * err_vel[i]) + (PID[1] * err_pos[i]) + (PID[2] * (err_vel[i] - prev_err_vel[i]));

    // Update velocity, applying firmware velocity limits
    step_vel[i] = MIN(MAX_SPD, MAX(-MAX_SPD, step_vel[i] + vel_adjust));
  
    // If step_vel is too small, we risk div by zero
    if (ABS(step_vel[i]) < MIN_SPD) {
      step_vel[i] = (step_vel[i] > 0) ? MIN_SPD : -MIN_SPD;
    }

    // ticks/step = (ticks/sec) * (sec / step)
    //            = (ticks/sec) * (1 / step_speed)
    ticks_per_step[i] = uint32_t(float(MOTION_WRITE_HZ) / abs(step_vel[i]));
    hal::stepDir(i, (step_vel[i] > 0) ? HIGH : LOW);    
  }

  hal::enableInterrupts();
  return true;
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

// Note: this is likely called inside a timer interrupt
// so care must be taken to keep overall cycles light.
// Avoid logging and other debugging calls
void motion::write() {
  ticks_since_last_print++;

  // Continue moving to target
  // Calculate ramp settings
  for (int i = 0; i < NUM_J; i++) {
    if (!active[i] || !limits_intended) {
      //dbg[2*i+1] = 'x';
      continue;
    }

    // Don't move if we're driving further into a limit
    uint8_t dir = (state::intent.pos[i] - state::actual.pos[i]) > 0;
    //dbg[2*i] = (dir) ? '+' : '-';
    if (!hal::readLimit(i) && !dir) {
      if (!(state::actual.mask[i] & MASK_LIMIT_TRIGGERED)) {
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
    hal::stepDn(i);
    steps_since_last_print++;
    if (state::intent.mask[i] & MASK_OPEN_LOOP_CONTROL) {
      state::actual.pos[i] += (step_vel[i] > 0) ? 1 : -1;
    }
    //dbg[2*i+1] = '.';
  }

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
