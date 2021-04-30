#include "app_hal.h"
#include <Encoder.h>

// Pins for step, step direction, and calibration (limit)
// This is device specific.

#ifdef AR3
const int STEP_PIN[] = {0, 2, 4, 6, 8, 10};
const int DIR_PIN[]  = {1, 3, 5, 7, 9, 11};
const int CAL_PIN[] = {26, 27, 28, 29, 30, 31};
const int STEP_EN = 0; // TODO

//set encoder pins
Encoder enc[] = {
  Encoder(14, 15),
  Encoder(16, 17),
  Encoder(18, 19),
  Encoder(20, 21),
  Encoder(22, 23),
  Encoder(24, 25)
};
#endif

#ifdef GSHIELD
const int STEP_PIN[] = {2, 3, 4};
const int DIR_PIN[] = {5, 6, 7};
const int CAL_PIN[] = {9, 10, 12};
const int STEP_EN = 8;

Encoder enc[] = {}; // TODO
#endif


namespace hal {

void init() {
  pinMode(STEP_EN, OUTPUT);
  digitalWrite(STEP_EN, HIGH); // Typically enable is active low

  for (int i = 0; i < NUM_J; i++) {
    // enc[i].write(START_POS[i] * ENC_MULT[i];
    pinMode(STEP_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    pinMode(CAL_PIN[i], INPUT_PULLUP);
    digitalWrite(STEP_PIN[i], HIGH);
  }
}

void stepDir(int i, bool dir) {
  digitalWrite(DIR_PIN[i], dir);
}
void stepDn(int i) {
  digitalWrite(STEP_PIN[i], LOW);
}
void stepUp(int i) {
  digitalWrite(STEP_PIN[i], HIGH);
}
bool readLimit(int i) {
  return digitalRead(CAL_PIN[i]);
}
int readEnc(int i) {
  return 0; // TODO
}
void writeEnc(int i, int value) {
  // TODO
}

} // namespace hal
