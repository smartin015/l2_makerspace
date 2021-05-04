#include "log.h"
#include "app_hal.h"
#include "comms.h"
#include "motion.h"
#include "state.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

void setup() {
  comms::init(); // Init comms first; may be needed to safely log outputs

  LOG_INFO("Setup begin for %d joint robot (report every %dms)", NUM_J, REPORT_PD_MILLIS);
  motion::init();
  hal::init();

  LOG_INFO("Configuring main timing loop for %d hz", MOTION_WRITE_HZ);
  hal::startMainTimer(MOTION_WRITE_HZ, &motion::write);

  LOG_INFO("Setup complete");
}

uint64_t last_report = 0;
uint8_t buf[128];
void loop() {
  uint64_t now = millis();
  if (now > last_report + REPORT_PD_MILLIS) {
    last_report = now;
    motion::print_state();
  }

  // NOTE: Casting directly to struct requires both the same endianness and same interpetation of floating point
  // nubmers. https://stackoverflow.com/questions/13775893/converting-struct-to-byte-and-back-to-struct
  int sz = comms::read(buf, sizeof(buf));
  if (sz > 0) {
    state::deserialize(&state::intent, buf);
    motion::intent_changed();
  }
  
  motion::read();

  if (motion::update()) {
    // LOG_INFO("SI0 %x %d %d SA0 %x %d %d", state::intent.mask[0], state::intent.pos[0], state::intent.vel[0], state::actual.mask[0], state::actual.pos[0], state::actual.vel[0]);
    state::serialize(buf, &state::actual);
    comms::write(buf, 2*(NUM_J*sizeof(int16_t)) + (NUM_J*sizeof(uint8_t)));
  }
}
