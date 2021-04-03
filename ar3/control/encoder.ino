/*
  Appears to be ~8k steps per rev
*/

// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:
int led = 13;
#define PRINT_INTERVAL 100
#define ENC_A1 22
#define ENC_B1 23

int enc = 0;
bool a = false;
bool b = false;

// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  pinMode(ENC_A1, INPUT);
  pinMode(ENC_B1, INPUT);
  Serial.begin(115200);
}

inline void processEncoder() {
  bool ea = digitalRead(ENC_A1);
  bool eb = digitalRead(ENC_B1);
  int da = ea - a;
  int db = eb - b;
  if (da != 0) {
    enc += da * (b ? -1 : 1);
  }
  if (db != 0) {
    enc += db * (a ? 1 : -1);
  }
  a = ea;
  b = eb;
}

// the loop routine runs over and over again forever:
unsigned long lastReport = 0;
void loop() {
  processEncoder();
  unsigned long t = millis();
  if (t - lastReport > PRINT_INTERVAL) {
    Serial.println(enc);
    lastReport = t;
  }
}
