#include "app_hal.h"
#include <Encoder.h>
#include "log.h"

// Pins for step, step direction, and calibration (limit)
// This is device specific.

#ifdef AR3
const int STEP_PIN[] = {0, 2, 4, 6, 8, 10};
const int DIR_PIN[]  = {1, 3, 5, 7, 9, 11};
const int CAL_PIN[] = {26, 27, 28, 29, 30, 31};
const int EN_PIN[] = {12, 12, 12, 12, 12, 12}; // TODO

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
#define PRESCALE 1 // TODO requires manual changes to timer setup code when changed
const int STEP_PIN[] = {2, 3, 4};
const int DIR_PIN[] = {5, 6, 7};
const int CAL_PIN[] = {9, 10, 12};
const int EN_PIN[] = {8, 8, 8}; // Shared enable pin

Encoder enc[] = {}; // TODO
#endif


namespace hal {

void init() {
  for (int i = 0; i < NUM_J; i++) {
    pinMode(EN_PIN[i], OUTPUT);
    stepEnabled(i, false);

    pinMode(STEP_PIN[i], OUTPUT);
    stepUp(i);

    pinMode(DIR_PIN[i], OUTPUT);
    stepDir(i, true);

    pinMode(CAL_PIN[i], INPUT_PULLUP);
    // enc[i].write(START_POS[i] * ENC_MULT[i];
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
void stepEnabled(int i, bool en) {
  digitalWrite(EN_PIN[i], (en) ? LOW : HIGH); // Enable is active low
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

void (*isr_cb)();
#ifdef AR3
#include <avr/io.h>
#include <avr/interrupt.h>
IntervalTimer teensyTimer;
#else
ISR(TIMER1_COMPA_vect) {
  isr_cb();
}
#endif // AR3

void startMainTimer(int hz, void (*cb)()) {
  // TODO version for teensy35
  disableInterrupts();
  isr_cb = cb;

#ifdef AR3
  // Teensy timer setup is handled via library
  teensyTimer.begin(isr_cb, 1000000 / hz);
#else 

  // Reset timer flags
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // freq = clock / (prescaler * (OCR1A + 1)) -> OCR1A = clock / (freq * prescaler) - 1
  int freq = F_CPU / (hz * PRESCALE) - 1;
  if (freq < (2 << sizeof(OCR1A))) {
    LOG_ERROR("Calculated frequency %d larger than OCR1A (%d bytes)", freq, sizeof(OCR1A));
  }
  OCR1A = freq;
  TCCR1B |= (1 << WGM12);   // CTC (clear timer on compare) mode, compare OCR1A
  TCCR1B |= (1 << CS10);    // No prescale, but enable
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
#endif // AR3
  enableInterrupts();
}

void disableInterrupts() {
  noInterrupts();
}
void enableInterrupts() {
  interrupts();
}

} // namespace hal
