#include "app_hal.h"
#include <Encoder.h>

//set encoder pins
Encoder enc[] = {
  Encoder(14, 15),
  Encoder(16, 17),
  Encoder(18, 19),
  Encoder(20, 21),
  Encoder(22, 23),
  Encoder(24, 25)
};

int readEnc(int idx) {
  return 0; // TODO
}

void writeEnc(int idx, int value) {
  // TODO
}
