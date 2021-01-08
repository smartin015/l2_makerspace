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

const int STEP_PIN[] = {0, 2, 4, 6, 8, 10};
const int DIR_PIN[]  = {1, 3, 5, 7, 9, 11};

int led = 13;
#define PRINT_INTERVAL 100
int last[NUM_J];
void setup() {
  pinMode(led, OUTPUT);
  for (int i = 0; i < NUM_J; i++) {
    jenc[i].write(0);
    last[i] = 0;
    pinMode(STEP_PIN[i], OUTPUT);
    digitalWrite(STEP_PIN[i], HIGH);
    pinMode(DIR_PIN[i], OUTPUT);
    digitalWrite(DIR_PIN[i], LOW);
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

#define US_DELAY 10
#define NSTEP 5
void pulse(int j, int dir) {
  Serial.print("Pulse ");
  Serial.print(j);
  Serial.print(" ");
  Serial.println(dir);
  digitalWrite(DIR_PIN[j], dir);
  for (int i = 0; i < NSTEP; i++) {
    digitalWrite(STEP_PIN[j], LOW);
    delayMicroseconds(US_DELAY);
    digitalWrite(STEP_PIN[j], HIGH);
    delayMicroseconds(US_DELAY);
  }
}

const char DN_MAP[NUM_J] = {'q','w','e','r','t','y'};

void cmd(char c) {
  if (c >= '1' && c <= '6') {
    return pulse(c - '1', LOW);
  } else {
    for (int i = 0; i < NUM_J; i++) {
      if (c == DN_MAP[i]) {
	return pulse(i, HIGH);
      }
    }
  }
}

unsigned long lastReport = 0;
void loop() {
  while (Serial.available()) {
    cmd(Serial.read());
  }

  unsigned long t = millis();
  if (t - lastReport < PRINT_INTERVAL) {
    return;
  }
  for (int i = 0; i < NUM_J; i++) {
    int v = jenc[i].read();
    Serial.print(dchar(v, last[i]));
    Serial.print(" ");
    Serial.print(v);
    Serial.print("\t");
    last[i] = v;
  }
  Serial.println();
  lastReport = t;
}
