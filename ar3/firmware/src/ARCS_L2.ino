#include "command.h"
#include "functions.h"

#include <avr/pgmspace.h>

String inData;
String function;
char WayPt[200][70];
int WayPtDel;


void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_J; i++) {
    enc[i].write(START_POS[i] * ENC_MULT[i])
    pinMode(STEP_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    pinMode(CAL_PIN[i], INPUT_PULLUP);
    digitalWrite(STEP_PIN[i], HIGH);
  }
}

void loop() {
  WayPtDel = 0;
  while (Serial.available() > 0 or WayPtDel == 1) {
    char recieved = Serial.read();
    inData += recieved;
    if (recieved != '\n') {
      continue;
    }
    command_t args = parse_command(inData);
    switch(args.function) {
      case FN_WAIT_TIME:
        fn_wait_time(args);
      case FN_GET_POS:
        fn_get_pos(args);
      case FN_CALIBRATE_ENC:
        fn_calibrate_enc(args);
      case FN_DRIVE_TO_LIMITS:
        fn_drive_to_limits(args);
      case FN_MOVE_J:
        fn_move_j(args);
      case FN_MOVE_L:
        fn_move_l(args);
      case FN_move_C:
        fn_move_c(args);
      default:
        printf("Unknown function: %s", f);
    }
    inData = "";
  }
}
