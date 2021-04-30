#include "comms.h"
#include <Arduino.h>

#define BUFLEN 128
uint8_t serbuf[BUFLEN];
int16_t idx = -1;
int16_t readlen = 0;

void comms::init() {
  Serial.begin(115200);
  Serial.setTimeout(500);
}

int comms::read(uint8_t* buf, int buflen) {
  char c;
  while (Serial.available()) {
    c = Serial.read();
    if (c == 0x79) {
      // Magic byte, next byte is length
      idx=0;
      readlen=0;
      continue;
    }
    if (idx == -1) {
      continue;
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
  return 0;
}

void comms::write(uint8_t* buf, int buflen) {
  Serial.write(0x79);
  Serial.write(buflen);
  Serial.write(buf, buflen);
}

static char pfbuf[128];
void comms::printf(char* format, ...) {
  va_list argptr;
  va_start(argptr, format);
  vsnprintf(pfbuf, sizeof(pfbuf), format, argptr);
  va_end(argptr);
  Serial.write(0x79);
  Serial.write(0x79);
  Serial.write(pfbuf, sizeof(pfbuf));
}

