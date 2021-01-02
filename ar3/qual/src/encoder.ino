/*
  Appears to be ~8k steps per rev
*/

#define NUM_J 6
#include <Encoder.h>
Encoder jenc[NUM_J] = {
  Encoder(14,15),
  Encoder(16,17),
  Encoder(18,19),
  Encoder(20,21),
  Encoder(22,23),
  Encoder(24,25),
};

int led = 13;
#define PRINT_INTERVAL 100
int last[NUM_J];
void setup() {
  pinMode(led, OUTPUT);
  for (int i = 0; i < NUM_J; i++) {
    jenc[i].write(0);
    last[i] = 0;
  }
  Serial.begin(115200);
  Serial.println("Init");
}

char dchar(int a, int b) {
  int d = a-b;
  if (d > 0) {
    return '^';
  } else if (d < 0) {
    return 'v';
  } else {
    return '=';
  }
}

unsigned long lastReport = 0;
void loop() {
  unsigned long t = millis();
  if (t - lastReport < PRINT_INTERVAL) {
    return;
  }
  for (int i = 0; i < NUM_J; i++) {
    int v = jenc[i].read();
    printf("%c%05d\t", dchar(v, last[i]), v);
    last[i] = v;
  }
  printf("\n");
  lastReport = t;
}
