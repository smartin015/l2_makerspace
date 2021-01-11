#include "functions.h"
#include "command.h"
#include "log.h"
#include "app_hal.h"

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

void do_fn(const command_t& args, char* out) {
  for (int i = 0; i < NFUNC; i++) {
    if (args.function[0] == COMMAND_ID[i][0] && args.function[1] == COMMAND_ID[i][1]) {
      cur_cmd = args;
      out_ptr = out;
      out_sz = 0;
      FUNCS[i](args);
    }
  }
}

int do_fn_complete() {
  return out_sz;
}


float cur_speed[NUM_J];
int next_pulse[NUM_J];
#define PD_MICROS 12
void loop_fn() {
  if (out_sz) {
    return; // Function completed 
  }

  if (cur_cmd.function[0] == 'M') {
    // Continue moving to target
    // Calculate ramp settings
    for (int i = 0; i < NUM_J; i++) {
      if (cur_cmd.step[i] <= 0) {
        continue;
      }
      // Check if we should pulse this joint
      next_pulse[i] -= PD_MICROS;
      if (next_pulse[i] <= 0) {
        // Speed is in pulses per second currently
        // TODO: Linear ramp up and down
        cur_cmd.step[i]--;
        next_pulse[i] = 1000000 / cur_cmd.extra[SPEED]; 
        digitalWrite(STEP_PIN[i], LOW);
      }
    }
  
    hal_usleep(5);

    bool move_complete = true;
    for (int i = 0; i < NUM_J; i++) {
      digitalWrite(STEP_PIN[i], HIGH);
      move_complete &= cur_cmd.step[i] <= 0;
      // TODO check for stalls
    }

    if (move_complete) {
      out_sz = sprintf(out_ptr, OK);
      /*
      // TODO write encoder positions
      for (int i = 0; i < NUM_J; i++) {
        writeEnc(i, T[i]);
      }
      */
    }
  }
}


void fn_wait_time(const command_t& args) {
  // delay(int(args.extra[SPEED] * 1000));
  LOG_DEBUG("Sleeping %02f", args.extra[SPEED]);
  hal_usleep(uint64_t(args.extra[SPEED]) * 1000000);
  out_sz = sprintf(out_ptr, OK);
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
}

void fn_calibrate_enc(const command_t& args) {
  for (int i = 0; i < NUM_J; i++) {
    writeEnc(i, args.step[i] * ENC_MULT[i]);
  }
  LOG_INFO("Encoders calibrated");
  out_sz = sprintf(out_ptr, OK);
}

void fn_drive_to_limits(const command_t& args) {
  int speed = int((SPEED_MULT * 2) / (args.extra[SPEED] / 100));
  for (int i = 0; i < NUM_J; i++) {
    // LOW when matching direction, HIGH when opposite direction
    digitalWrite(DIR_PIN[i], ROT_DIR[i] ^ args.enc[i]);
  }
  
  //drive motors for calibration
  int done[NUM_J];
  bool move_complete[NUM_J];
  bool all_complete = false;
  while (!all_complete) {
    all_complete = true;
    for (int i = 0; i < NUM_J; i++) {
      move_complete[i] = !((done[i] < args.step[i]) && (digitalRead(CAL_PIN[i]) == LOW));
      all_complete &= move_complete[i];
      digitalWrite(STEP_PIN[i], !move_complete);
    }
    hal_usleep(5);
    for (int i = 0; i < NUM_J; i++) {
      digitalWrite(STEP_PIN[i], LOW);   
      done[i]++;
    } 
    hal_usleep(speed);
  }
  
  // Overdrive steppers to ensure good contact
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

  hal_usleep(500000);

  // Fail if any switches not made
  for(int i = 0; i < NUM_J; i++) {
    if (!digitalRead(CAL_PIN[i])) {
      out_sz = sprintf(out_ptr, "F");
    }
  }
  out_sz = sprintf(out_ptr, "P");
}

void fn_move_j(const command_t& args) {
  //find highest step & set direction bits
  /*
  int high = 0;
  for (int i = 0; i < NUM_J; i++) {
    if (args.step[i] > high) {
      high = args.step[i];
    }
    digitalWrite(DIR_PIN[i], dir[i] ^ rotDir[i]);
  }
  */
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
}
