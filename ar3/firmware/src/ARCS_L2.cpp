#include "log.h"
#include "app_hal.h"
#include "command.h"
#include "comms.h"
#include "functions.h"

// #include <avr/pgmspace.h>

String inData;
String function;
// char WayPt[200][70];
// int WayPtDel;

void setup() {
  initComms();
  // Serial.begin(115200);
  for (int i = 0; i < NUM_J; i++) {
    // enc[i].write(START_POS[i] * ENC_MULT[i])
    pinMode(STEP_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    pinMode(CAL_PIN[i], INPUT_PULLUP);
    digitalWrite(STEP_PIN[i], HIGH);
  }
  LOG_INFO("Setup complete");
}

char buf[128];
void loop() {
  uint8_t n = do_fn_complete();
  if (n) {
    LOG_DEBUG("Send response %s", buf);
    sendResponse(buf, n);
  }
  if (do_fn_ready()) {
    if (tryFetchCommand(buf, 128)) {
      command_t args = parse_command(buf);
      do_fn(args, buf);
    }
  }
  loop_fn();
}
