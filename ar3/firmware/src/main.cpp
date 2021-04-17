#include "log.h"
#include "app_hal.h"
#include "comms.h"
#include "motion.h"
#include "state.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

void setup() {
  comms::init();
  motion::init();
  for (int i = 0; i < NUM_J; i++) {
    hal::initJoint(i);
  }
  LOG_INFO("Setup complete");
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
    state::deserialize(&state::intent, buf);
    motion::intent_changed();
  }
  motion::read();
  motion::update();
  motion::write();
  if (sz > 0) {
    // Comms follows ZMQ req/rep communication; exactly one reply per received request
    // LOG_INFO("SI0 %x %d %d SA0 %x %d %d", state::intent.mask[0], state::intent.pos[0], state::intent.vel[0], state::actual.mask[0], state::actual.pos[0], state::actual.vel[0]);
    state::serialize(buf, &state::actual);
    comms::write(buf, 2*(NUM_J*sizeof(int16_t)) + (NUM_J*sizeof(uint8_t)));
  }
}
