#include "app_hal.h"
#include "functions.h"
#include "command.h"
#include "log.h"

typedef void (*fn)(const command_t& args);

const char* UNIMPL = "UNIMPLEMENTED";
const char* OK = "OK";


const char* COMMAND_ID[NFUNC] = {"WT", "GP", "LM", "LL", "MJ", "ML", "MC"};
void fn_wait_time(const command_t& args);
void fn_get_pos(const command_t& args);
void fn_calibrate_enc(const command_t& args);
void fn_drive_to_limits(const command_t& args);
void fn_move_j(const command_t& args);
void fn_move_l(const command_t& args);
void fn_move_c(const command_t& args);
const fn FUNCS[NFUNC] = {fn_wait_time, fn_get_pos, fn_calibrate_enc, fn_drive_to_limits, fn_move_j, fn_move_l, fn_move_c};

command_t cur_cmd;
char* out_ptr = nullptr;
int out_sz = 0;
bool ready = true;

// Arm state machine:
// UNKNOWN ---(init)-------> WAITING
//
// WAITING ---(cmd recv)---> MOVING
//
// MOVING  ---(limits hit)----> COMPLETE (with err text)
//         |--(LL, all lims)--> OVERDRIVE
//         |--(finished)------> COMPLETE
//
// OVERDRIVE ---(finished)----> BACKING
// BACKING -----(finished)----> COMPLETE
enum fn_state_t {
  UNKNOWN = 0,
  WAITING = 1,
  MOVING = 2,
  OVERDRIVING = 3,
  BACKING = 4,
  COMPLETE = 5,
};
fn_state_t fn_state = UNKNOWN;

void do_fn(const command_t& args, char* out) {
  LOG_DEBUG("do_fn %c%c", args.function[0], args.function[1]);
  for (int i = 0; i < NFUNC; i++) {
    if (args.function[0] == COMMAND_ID[i][0] && args.function[1] == COMMAND_ID[i][1]) {
      cur_cmd = args;
      out_ptr = out;
      out_sz = 0;
      FUNCS[i](args);
      return;
    }
  }
}

bool do_fn_ready() {
  return fn_state == WAITING;
}

int do_fn_complete() {
  if (fn_state == COMPLETE) {
    int sz = out_sz;
    out_sz = 0;
    fn_state = WAITING;
    return sz;
  }
  return 0;
}

float cur_speed[NUM_J];
uint64_t next_pulse[NUM_J];
void continue_moving() {
  // Stop early if we hit a limit
  for(int i = 0; i < NUM_J; i++) {
    if (!digitalRead(CAL_PIN[i])) {
      out_sz = sprintf(out_ptr, "LIMIT %d", i+1);
      cur_cmd.function[0] = '\0';
      fn_state = COMPLETE;
    }
  }

  // Continue moving to target
  // Calculate ramp settings
  uint64_t now = millis(); // TODO micros?
  for (int i = 0; i < NUM_J; i++) {
    if (cur_cmd.step[i] == 0) {
      continue;
    }
    // Check if we should pulse this joint
    if (now > next_pulse[i]) {
      // Speed is in pulses per second currently
      // TODO: Linear ramp up and down
      // TODO handle negative stepping
      cur_cmd.step[i] += (cur_cmd.step[i] > 0) ? -1 : 1;
      next_pulse[i] += 1000 / cur_cmd.extra[SPEED]; 
      digitalWrite(STEP_PIN[i], LOW);
    }
  }

  hal_usleep(5);

  bool move_complete = true;
  for (int i = 0; i < NUM_J; i++) {
    digitalWrite(STEP_PIN[i], HIGH);
    move_complete &= (cur_cmd.step[i] == 0);
    // TODO check for stalls
  }

  hal_usleep(5000);

  if (move_complete) {
    out_sz = sprintf(out_ptr, OK);
    fn_state = COMPLETE;
    /*
    // TODO write encoder positions
    for (int i = 0; i < NUM_J; i++) {
      writeEnc(i, T[i]);
    }
    */
  }
}

void continue_overdrive() {
  // Overdrive steppers to ensure good contact
  /*
  for (int n = 0; n < OVERDRIVE_STEPS; n++) {
    for (int i = 0; i < NUM_J; i++) {
      if (args.step[i] > 0) {
        digitalWrite(STEP_PIN[i], LOW);
      }
    }
    hal_usleep(5);
    for (int i = 0; i < NUM_J; i++) {
      if (args.step[i] > 0) {
        digitalWrite(STEP_PIN[i], HIGH);
      }
    }
    hal_usleep(speed);
  }
  */

  hal_usleep(500000);
  out_sz = sprintf(out_ptr, "UNIMPLEMENTED");
  fn_state = COMPLETE;
}

void continue_backing() {
  /*
  // Fail if any switches not made
  for(int i = 0; i < NUM_J; i++) {
    if (!digitalRead(CAL_PIN[i])) {
      out_sz = sprintf(out_ptr, "F");
    }
  }
  out_sz = sprintf(out_ptr, "P");
  */
  out_sz = sprintf(out_ptr, "UNIMPLEMENTED");
  fn_state = COMPLETE;
}

#define PD_MICROS 12
void loop_fn() {
  if (out_sz || ready) {
    return; // Function complete or not started
  }

  switch (fn_state) {
    case UNKNOWN:
    case WAITING:
    case COMPLETE:
      return;
    case MOVING:
      continue_moving();
    case OVERDRIVING:
      continue_overdrive();
    case BACKING:
      continue_backing();
    default:
      LOG_ERROR("Unknown fn_state %d", fn_state);
  }
}


void fn_wait_time(const command_t& args) {
  LOG_DEBUG("Sleeping %02f", args.extra[SPEED]);
  // TODO sleep without blocking
  hal_usleep(uint64_t(args.extra[SPEED]) * 1000000);
  out_sz = sprintf(out_ptr, OK);
  fn_state = COMPLETE;
}

void fn_get_pos(const command_t& args) {
  char errors[NUM_J+1];
  errors[NUM_J] = '\0';
  int values[NUM_J];
  bool hasError = false;
  for (int i = 0; i < NUM_J; i++) {
    int curStep = readEnc(i) / ENC_MULT[i];
    if (abs(curStep - args.step[i]) > ENC_MULT[i] / ENC_DIV) {
      errors[i] = '1';
      values[i] = 0;
      hasError = true;
    } else {
      errors[i] = '0';
      values[i] = curStep;
      writeEnc(i, args.step[i] * ENC_MULT[i]);
    }
  }
  out_sz = sprintf(out_ptr, "%s%s" "%c%d" "%c%d" "%c%d" "%c%d" "%c%d" "%c%d", 
    (hasError) ? "01" : "00", errors, 
    ENC_CHAR[0], values[0],
    ENC_CHAR[1], values[1],
    ENC_CHAR[2], values[2],
    ENC_CHAR[3], values[3],
    ENC_CHAR[4], values[4],
    ENC_CHAR[5], values[5]
  );
  fn_state = COMPLETE;
}

void fn_calibrate_enc(const command_t& args) {
  for (int i = 0; i < NUM_J; i++) {
    writeEnc(i, args.step[i] * ENC_MULT[i]);
  }
  LOG_INFO("Encoders calibrated");
  out_sz = sprintf(out_ptr, OK);
  fn_state = COMPLETE;
}

void fn_drive_to_limits(const command_t& args) {
  uint64_t now = millis();
  for (int i = 0; i < NUM_J; i++) {
    // LOW when matching direction, HIGH when opposite direction
    digitalWrite(DIR_PIN[i], ROT_DIR[i] ^ args.enc[i]);
    next_pulse[i] = now;
  }
  fn_state = MOVING;
}

void fn_move_j(const command_t& args) {
  //find highest step & set direction bits
  LOG_DEBUG("Setting targets for MOVE J - %d %d %d %d %d %d", 
    args.step[0], args.step[1], args.step[2], args.step[3], args.step[4], args.step[5]);
  int high = 0;
  uint64_t now = millis();
  for (int i = 0; i < NUM_J; i++) {
    if (args.step[i] > high) {
      high = args.step[i];
    }
    digitalWrite(DIR_PIN[i], (args.step[i] > 0) ^ ROT_DIR[i]);
    next_pulse[i] = now;
  }
  fn_state = MOVING;
}
      
void fn_move_l(const command_t& args) {
  /*
  WayPtDel = 1;
  int NumPtsStart = inData.indexOf('L');
  int WayPts = inData.substring(NumPtsStart + 1).toInt();
  LOG_DEBUG();
  inData = ""; // Clear recieved buffer
  //STORE WAYPOINTS
  int i = 0;
  while (i < WayPts) {
    while (Serial.available() > 0) {
      char recieved = Serial.read();
      inData += recieved;
      if (recieved == '\n') {
        inData.toCharArray(WayPt[i], 70);
        LOG_DEBUG();
        ++i;
        inData = ""; // Clear recieved buffer
      }
    }
  }
  LOG_DEBUG();
  //EXECUTE WAYPOINTS

  i = 0;
  while (i <= WayPts+1) {
    inData = WayPt[i];
    // TODO execute  fn_move_j() without motor stall check
    ++i;
  }
  // TODO motor stall check
  */
  LOG_DEBUG("UNIMPLEMENTED");
  out_sz = sprintf(out_ptr, UNIMPL);
  fn_state = COMPLETE;
}

void fn_move_c(const command_t& args) {
  /*
  WayPtDel = 1;
  int NumPtsStart = inData.indexOf('C');
  int WayPts = inData.substring(NumPtsStart + 1).toInt();
  LOG_DEBUG();
  inData = ""; // Clear recieved buffer
  //STORE WAYPOINTS
  int i = 0;
  while (i < WayPts) {
    while (Serial.available() > 0) {
      char recieved = Serial.read();
      inData += recieved;
      if (recieved == '\n') {
        inData.toCharArray(WayPt[i], 70);
        LOG_DEBUG();
        ++i;
        inData = ""; // Clear recieved buffer
      }
    }
  }
  LOG_DEBUG();
  i = 0;
  while (i < WayPts+1) {
    inData = WayPt[i];
    // TODO fn_move_j without stall check
    ++i;
  }

  // TODO stall check
  WayPtDel = 0;
  LOG_DEBUG();
  */
  LOG_DEBUG("UNIMPLEMENTED");
  out_sz = sprintf(out_ptr, UNIMPL);
  fn_state = COMPLETE;
}
