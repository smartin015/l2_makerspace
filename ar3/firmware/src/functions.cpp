#include "app_hal.h"
#include "state.h"
#include "functions.h"
#include <cstdio>

void write_outputs() {

  // Continue moving to target
  // Calculate ramp settings
  uint64_t now = millis(); // TODO micros?
  for (int i = 0; i < NUM_J; i++) {
    // Don't move if we hit a limit
    if (!digitalRead(CAL_PIN[i])) {
      printf("Limit %d hit; skipping move\n", i);
      continue;
    }

    digitalWrite(DIR_PIN[i], (state::intent.pos[i] - state::actual.pos[i] > 0) ^ ROT_DIR[i]);

    // Check if we should pulse this joint
    // TODO implement - factor in pulse speed vs velocity
    if (state::intent.pos[i] - state::actual.pos[i] != 0) {
      digitalWrite(STEP_PIN[i], LOW);
    }
  }

  hal_usleep(5);

  for (int i = 0; i < NUM_J; i++) {
    digitalWrite(STEP_PIN[i], HIGH);
  }
}

void read_inputs() {
  int values[NUM_J];
  bool hasError = false;
  int ts = 1; // TODO
  for (int i = 0; i < NUM_J; i++) {
    int p = readEnc(i);
    state::actual.vel[i] = (p - state::actual.pos[i]) / ts;
    state::actual.pos[i] = p;
  }
}

void set_encoders(const int values[NUM_J]) {
  for (int i = 0; i < NUM_J; i++) {
    writeEnc(i, values[i]);
  }
}
