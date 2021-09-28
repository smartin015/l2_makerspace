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

// To safeguard against e.g. integer overflow causing the robot to lose control,
// any computed P, I, or D contributions beyond this limit trigger an emergency
// deceleration of the robot that can only be reset by cycling power to the robot.
// It is assumed that the robot can be power cycled in a safe manner.
#define HARD_MAX_PID_CONTRIBUTION 100000

// Set this wide enough that the step pin interrupt
// can be triggered on the stepper driver - see driver 
// manual for acceptable limits.
// Default: 5 us
#define STEP_PIN_WRITE_USEC 5

// char dbg[2*NUM_J] = "";
bool should_check_limits;
bool emergency_decel_triggered;
bool limit_triggered[NUM_J];
float step_vel[NUM_J];
uint32_t ticks_per_step[NUM_J];
uint32_t ticks_per_step_smoothed[NUM_J]; // interpolate between ticks_per_step values to prevent judder
uint32_t ticks[NUM_J];
uint32_t ticks_since_last_print = 0;
uint32_t steps_since_last_print = 0; // Cumulative, all joints
uint32_t msgs_received_since_last_print = 0;

uint64_t last_velocity_update = 0;
float prev_vel[NUM_J];
float prev_err_vel[NUM_J];
int prev_pos[NUM_J];
int err_pos[NUM_J];
float err_vel[NUM_J];
float pid_updates[NUM_J][3];
bool active[NUM_J];

void motion::init() {
  for (int i = 0; i < NUM_J; i++) {
    emergency_decel_triggered = false;
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
    for (int j = 0; j < 3; j++) {
      pid_updates[i][j] = 0;
    }
  }
}

void motion::print_state() {
  // NOTE: arduino doesn't include floating point printf by default; scale & cast floats
  LOG_DEBUG("rx %u, dt %u\tds %u\tactive %s\twant m%02x p%d v%d\tgot m%02x p%d v%d --> stepvel %d ticks/step %d", 
      msgs_received_since_last_print,
      ticks_since_last_print,
      steps_since_last_print,
      (active[0]) ? "Y" : "N",
      state::intent.mask[0], state::intent.pos[0], int(state::intent.vel[0]*100),
      state::actual.mask[0], state::actual.pos[0], int(state::actual.vel[0]*100),
      int(step_vel[0]*100), ticks_per_step[0]);
  ticks_since_last_print = 0;
  steps_since_last_print = 0;
  msgs_received_since_last_print = 0;
}

void motion::print_pid_stats() {
  LOG_DEBUG("P %d\tI %d\tD %d\t -> %d + %d + %d",
    int(state::settings.pid[0]*100), int(state::settings.pid[1]*100), int(state::settings.pid[2]*100),
    int(pid_updates[0][0]*100), int(pid_updates[0][1]*100), int(pid_updates[0][2]*100));
}

// We have a separate motion function for deceleration (with some duplication of logic vs e.g. motion::update()) to ensure 
// changes to motion execution do not affect the safe stopping of the robot.
bool emergency_decel(int dt) {
  hal::disableInterrupts();
  for (int i = 0; i < NUM_J; i++) {
    if (!active[i]) {
      continue;
    }

    // Decelerate maximally in the direction that gets us towards zero speed.
    float max_accel = MAX(DEFAULT_MAX_ACCEL, state::settings.max_accel); // Don't assume max_accel setting is big enough (or even positive)
    float vel_adjust = ((step_vel[i] > 0) ? -1 : 1) * float(max_accel) * 1000 / dt;
    if (ABS(vel_adjust) > ABS(step_vel[i])) {
      step_vel[i] = 0;
    } else {
      step_vel[i] += vel_adjust;
    }

    // Deactivate steppers when low speed reached
    float initial_spd = MAX(DEFAULT_INITIAL_SPD, state::settings.initial_spd); // Don't assume initial_spd setting is nonzero / non-negative
    if (ABS(step_vel[i]) < initial_spd) {
      active[i] = false;
      continue;
    }

    // ticks/step = (ticks/sec) * (sec / step)
    //            = (ticks/sec) * (1 / step_speed)
    ticks_per_step[i] = uint32_t(float(MOTION_WRITE_HZ) / ABS(step_vel[i]));
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

// Velocities are implemented by slowly adjusting 
// the stepping period for each joint based on the target
// velocity and (currently hardcoded) acceleration profile given for each motor
bool motion::update() {
  uint64_t now = millis();
  int dt = now - last_velocity_update;

  if (should_check_limits) {
    // Not exactly part of motion planning - run this here instead of in write()
    // to keep the interrupt lightweight
    for (int i = 0; i < NUM_J; i++) {
      if (!active[i]) {
        continue;
      }
      // Don't move if we're driving further into a limit
      uint8_t dir = (state::intent.pos[i] - state::actual.pos[i]) > 0;
      //dbg[2*i] = (dir) ? '+' : '-';
      if (!hal::readLimit(i) && !dir) {
        if (!(state::actual.mask[i] & MASK_LIMIT_TRIGGERED)) {
          state::actual.mask[i] |= MASK_LIMIT_TRIGGERED;
        }
        recalc_limit_intent();
      } else if (state::actual.mask[i] & MASK_LIMIT_TRIGGERED) {
        // Update actual limit state and per-joint limit state
        state::actual.mask[i] &= ~MASK_LIMIT_TRIGGERED;
        recalc_limit_intent();
      }
    }
  }


  if (dt < state::settings.velocity_update_pd_millis) {
    return false;
  }
  last_velocity_update = now;

  if (emergency_decel_triggered) {
    return emergency_decel(dt);
  }

  if (dt > 2 * state::settings.velocity_update_pd_millis) {
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
    active[i] = (ABS(err_vel[i]) > VEL_DEAD_ZONE) || (ABS(err_pos[i]) > POS_DEAD_ZONE);
    for (int i = 0; i < NUM_J; i++) {
    	bool en = state::intent.mask[i] & MASK_ENABLED; 
	// We *can* be enabled, but we may not *need* to be enabled.
	hal::stepEnabled(i, en && active[i]);
    }
    if (!active[i]) {
      continue;
    }
    
    // This is PID adjustment targeting velocity - in this case, P=velocity, I=position, D=acceleration
    // Note that we have targets both for position and velocity, but not for acceleration - that's where
    // we use a real value.
    pid_updates[i][0] = state::settings.pid[0] * err_vel[i];
    pid_updates[i][1] = state::settings.pid[1] * err_pos[i];
    pid_updates[i][2] = state::settings.pid[2] * (err_vel[i] - prev_err_vel[i]);
    if (pid_updates[i][0] > HARD_MAX_PID_CONTRIBUTION ||
        pid_updates[i][1] > HARD_MAX_PID_CONTRIBUTION ||
        pid_updates[i][2] > HARD_MAX_PID_CONTRIBUTION) {
      emergency_decel_triggered = true;
      LOG_ERROR("PID UPDATE %d %d %d CONTAINS AT LEAST 1 TERM ABOVE HARD LIMIT %d - EMERGENCY DECELERATION ENABLED", 
          int(pid_updates[i][0]), int(pid_updates[i][1]), int(pid_updates[i][2]), HARD_MAX_PID_CONTRIBUTION);
      return false;
    }

    float vel_adjust = (pid_updates[i][0] + pid_updates[i][1] + pid_updates[i][2]);

    // Acceleration is limited
    vel_adjust = MIN(float(state::settings.max_accel) * 1000 / dt, MAX(-float(state::settings.max_accel) * 1000 / dt, vel_adjust));
      
    // Update velocity, applying velocity limits
    step_vel[i] = MIN(state::settings.max_spd, MAX(-state::settings.max_spd, step_vel[i] + vel_adjust));
  
    // If step_vel is too small, we risk div by zero
    if (ABS(step_vel[i]) < state::settings.initial_spd) {
      step_vel[i] = (step_vel[i] > 0) ? state::settings.initial_spd : -state::settings.initial_spd;
    }

    // ticks/step = (ticks/sec) * (sec / step)
    //            = (ticks/sec) * (1 / step_speed)
    ticks_per_step[i] = uint32_t(float(MOTION_WRITE_HZ) / ABS(step_vel[i]));
    hal::stepDir(i, (step_vel[i] > 0) ? HIGH : LOW);    
  }

  hal::enableInterrupts();
  return true;
}

void motion::intent_changed() {
  msgs_received_since_last_print++;
  recalc_limit_intent();

  for (int i = 0; i < NUM_J; i++) {
    bool en = state::intent.mask[i] & MASK_ENABLED; 
    // We *can* be enabled, but we may not *need* to be enabled.
    hal::stepEnabled(i, en && active[i]);
    state::actual.mask[i] = (state::actual.mask[i] & ~MASK_ENABLED) | (en ? MASK_ENABLED : 0);
  }
}

// Note: this is likely called inside a timer interrupt
// so care must be taken to keep overall cycles light.
// Avoid logging and other debugging calls
void motion::write() {
  ticks_since_last_print++;

  // Check limits less frequently than stepping
  if (ticks_since_last_print % LIMIT_CHECK_TICK_PD == 0) {
    should_check_limits = true;
  }

  // Apply some smoothing between current stepping and intent
  if (ticks_since_last_print % STEP_INTERPOLATION_TICK_PD == 0) {
    // Interpolation is done in the frequency domain, not the time domain, for linear speed response
    for (uint8_t i = 0; i < NUM_J; i++) {
      uint32_t uhz_real = (1000000 / ticks_per_step[i]);
      uint32_t uhz_smooth = (1000000 / ticks_per_step_smoothed[i]);
      uint32_t uhz_new = (uhz_real + (uhz_smooth*15))/16; // Moving average
      if (uhz_new != 0) {
	      ticks_per_step_smoothed[i] = (1000000 / uhz_new);
      }
    }
  }

  // Continue moving to target
  // Calculate ramp settings
  for (uint8_t i = 0; i < NUM_J; i++) {
    if (!active[i] || !limits_intended) {
      continue;
    }

    if (ticks[i] == 0) {
      ticks[i] = ticks_per_step[i];
      hal::stepDn(i);
      steps_since_last_print++;
      if (state::intent.mask[i] & MASK_OPEN_LOOP_CONTROL) {
        state::actual.pos[i] += (step_vel[i] > 0) ? 1 : -1;
      }
    } else {
      ticks[i]--;
    }
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
