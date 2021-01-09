#include "functions.h"
#include "command.h"
#include "log.h"

typedef void (*fn)(const command_t& args);

const char* COMMAND_ID[NFUNC] = {"WT", "GP", "LM", "LL", "MJ", "ML", "MC"};
const fn FUNCS[NFUNC] = {fn_wait_time, fn_get_pos, fn_calibrate_enc, fn_drive_to_limits, fn_move_j, fn_move_l, fn_move_c};

bool do_fn(const command_t& args) {
  for (int i = 0; i < NFUNC; i++) {
    if (args.function[0] == COMMAND_ID[i][0] && args.function[1] == COMMAND_ID[i][1]) {
      FUNCS[i](args);
      return true;
    }
  }
  return false;
}


void fn_wait_time(const command_t& args) {
  // delay(int(args.extra[SPEED] * 1000));
  LOG_DEBUG("Done");
}

void fn_get_pos(const command_t& args) {
  /*
  char errors[NUM_J+1];
  errors[NUM_J] = '\0';
  String values = "";
  bool hasError = false;
  for (int i = 0; i < NUM_J; i++) {
    int curStep = enc[i].read() / ENC_MULT[i];
    if (abs(curStep[i] - args.step[i]) > ENC_MULT[i] / ENC_DIV) {
      errors[i] = '1';
      hasError = true;
      values += ENC_CHAR[i] + "0";
    } else {
      errors[i] = '0';
      values += ENC_CHAR[i] + itoa(curStep[i]);
      enc[i].write(args.step[i] * ENC_MULT[i]);
    }
  }
  printf("%s%s%s", (hasError) ? "01" : "00", errors, values);
  */
}

void fn_calibrate_enc(const command_t& args) {
  /*
  for (int i = 0; i < NUM_J; i++) {
    enc.write(args.step[i] * ENC_MULT[i]);
  }
  */
  LOG_DEBUG("Done");
}

void fn_drive_to_limits(const command_t& args) {
  /*
  int speed = int((SPEED_MULT * 2) / (args.speed / 100));
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
    // delayMicroseconds(5);
    for (int i = 0; i < NUM_J, i++) {
      digitalWrite(STEP_PIN[i], LOW);   
    } 
    done[i]++;
    // delayMicroseconds(speed);
  }
  
  // Overdrive steppers to ensure good contact
  for (int n = 0; n < OVERDRIVE_STEPS) {
    for (int i = 0; i < NUM_J; i++) {
      if (args.step[i] > 0) {
        digitalWrite(STEP_PIN[i], LOW);
      }
    }
    // delayMicroseconds(5);
    for (int i = 0; i < NUM_J; i++) {
      if (step[i] > 0) {
        digitalWrite(STEP_PIN[i], HIGH);
      }
    }
    // delayMicroseconds(speed);
  }

  // delay(500);

  // Fail if any switches not made
  for(int i = 0; i < NUM_J; i++) {
    if (!digitalRead(CAL_PIN[i])) {
      printf("F\r");
      return;
    }
  }
  printf("P\r");
  */
}

void fn_move_j(const command_t& args) {
  //find highest step & set direction bits
  /*
  int high = 0;
  for (int i = 0; i < NUM_J; i++) {
    high = max(high, args.step[i]);
    digitalWrite(DIR_PIN[i], dir[i] ^ rotDir[i]);
  }
  
  // Calculate ramp settings
  float hold_start = high * args.extra[ACCEL_DURATION];
  float hold_end = high - (high * args.extra[DECEL_DURATION]);
  float hold_period = int(SPEED_MULT / args.extra[SPEED]);
  float extra_period = int(SPEED_MULT / args.extra[SPEED]); // TODO parse this
  float move_end = high;
  float accel = hold_speed / hold_start;
  float decel = hold_speed / (high - hold_end);

  // ramp up, hold, and down
  float cur_delay = 0;
  float step_rate[NUM_J]; // Multiplier on longest distance
  for (int i = 0; i < NUM_J; i++) {
   step_rate[i] = args.step[i] / high;
  }
  int cur[NUM_J];
  for (int t = 0; t < move_end; highCur++) {
    cur_delay = hold_period;
    if (highCur <= hold_start) {
      cur_delay += extra_period - max(extra_period, accel * t);
    } else if (highCur >= hold_end) {
      cur_delay += max(extra_period, decel * (t - hold_end));
    }
    // Step each joint at their given rate
    // TODO: Check encoders while stepping
    for (int i = 0; i < NUM_J; i++) {
      if (step_rate[i] * t > cur[i]) {
        digitalWrite(STEP_PIN[i], LOW);
        delayMicroseconds(1);
        digitalWrite(STEP_PIN[i], HIGH);
        cur[i]++;
      }
    }
  }

  // check for stalls
  char error[NUM_J];
  bool hasError = false;
  String result;
  for (int i = 0; i < NUM_J; i++) {
    curStep[i] = enc[i].read() / ENC_MULT[i];
    if (abs(curStep[i] - tarStep[i]) > ENC_MULT[i] / ENC_DIV[i]) {
      error[i] = '1';
      hasError = true;
    } else {
      error[i] = '0';
    }     
    result += ENC_CHAR[i] + itoa(curStep[i]);
  }

  if (hasError) {
    printf("%s%s%s", (hasError) ? "01" : "00", error, result);
  } else {
    for (int i = 0; i < NUM_J; i++) {
      enc[i].write(tarStep[i] * ENC_MULT[i]);
    }
  }
  LOG_DEBUG();
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
}
