#include "comms.h"
#include <Arduino.h>

#define BUFLEN 128
char serbuf[BUFLEN];
size_t idx = 0;

void initComms() {
  Serial.begin(115200);
}

bool tryFetchCommand(char* buf, size_t buflen) {
  char c;
  while (Serial.available()) {
    c = Serial.read();
    serbuf[idx++] = c;
    if (c == '\n') {
      strncpy(buf, buflen, serbuf);
      return true;
    } else if (idx >= BUFLEN) {
      idx = 0;
      // TODO overrun warning indicator
    }
  }
  return false;
}
