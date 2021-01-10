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
  initHAL();
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
  if (tryFetchCommand(buf, 128)) {
    command_t args = parse_command(buf);
    uint8_t n = do_fn(args, buf);
    if (n) {
      sendResponse(buf, n);
    } else {
      LOG_ERROR("Unknown function: %s", args.function);
    }
  }
}
