#include "comms.h"
#include <Arduino.h>

#define BUFLEN 128
uint8_t serbuf[BUFLEN];
int16_t idx = -1;
int16_t readlen = 0;

void comms::init() {
  Serial.begin(115200);
}

int comms::read(uint8_t* buf, int buflen) {
  char c;
  while (Serial.available()) {
    c = Serial.read();
    if (idx == -1) {
      continue;
    }
    if (c == 0x79) {
      // Magic byte, next byte is length
      idx=0;
      readlen=0;
    }
    if (readlen == 0) {
      // NOTE: Max length is 255 characters
      readlen = c;
      continue;
    }
    serbuf[idx++] = c;
    if (idx == readlen) {
      memcpy(buf, serbuf, readlen);
      idx = -1;
      return readlen;
    } else if (idx >= BUFLEN) {
      idx = -1;
      // TODO overrun warning indicator
    }
  }
  return false;
}

void comms::write(uint8_t* buf, int buflen) {
  Serial.write(0x79);
  Serial.write(buflen);
  Serial.write(buf, buflen);
}
