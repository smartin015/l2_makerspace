#include "log.h"
#include "app_hal.h"
#include "comms.h"
#include "functions.h"
#include "state.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

void setup() {
  comms::init();
  for (int i = 0; i < NUM_J; i++) {
    // enc[i].write(START_POS[i] * ENC_MULT[i])
    pinMode(STEP_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    pinMode(CAL_PIN[i], INPUT_PULLUP);
    digitalWrite(STEP_PIN[i], HIGH);
  }
  LOG_INFO("Setup complete");
}

void serialize(uint8_t* buf, state::state_t* state) {
  for (int i = 0; i < NUM_J; i++) {
    buf[i] = state->mask[i];
    buf[NUM_J + 2*i] = state->pos[i] & 0xff;
    buf[NUM_J + 2*i + 1] = (state->pos[i] >> 8) & 0xff;
    buf[3*NUM_J + 2*i] = state->vel[i] & 0xff;
    buf[3*NUM_J + 2*i + 1] = (state->vel[i] >> 8) & 0xff;
  }
}

void deserialize(state::state_t* state, uint8_t* buf) {
  for (int i = 0; i < NUM_J; i++) {
    state->mask[i] = buf[i];
    state->pos[i] = buf[NUM_J + 2*i] + (buf[NUM_J + 2*i + 1] << 8);
    state->vel[i] = buf[3*NUM_J + 2*i] + (buf[3*NUM_J + 2*i + 1] << 8);
    // printf("J%d %d %x %d %x -> %d\n", i, NUM_J + 2*i, buf[NUM_J + 2*i], NUM_J + 2*i+1, buf[NUM_J + 2*i+1], state->pos[i]);
  }
}

uint8_t buf[128];
void loop() {
  // NOTE: Casting directly to struct requires both the same endianness and same interpetation of floating point
  // nubmers. https://stackoverflow.com/questions/13775893/converting-struct-to-byte-and-back-to-struct
  int sz = comms::read(buf, sizeof(buf));
  if (sz != 0 && sz != 30) {
    LOG_INFO("BAD SIZE %d", sz);
    return;
  }
  if (sz > 0) {
    deserialize(&state::intent, buf);
  }
  read_inputs();
  write_outputs();
  if (sz > 0) {
    // Comms follows ZMQ req/rep communication; exactly one reply per received request
    // LOG_INFO("SI0 %x %d %d SA0 %x %d %d", state::intent.mask[0], state::intent.pos[0], state::intent.vel[0], state::actual.mask[0], state::actual.pos[0], state::actual.vel[0]);
    serialize(buf, &state::actual);
    comms::write(buf, 2*(NUM_J*sizeof(int16_t)) + (NUM_J*sizeof(uint8_t)));
  }
  usleep(1 * 1000000);
}
