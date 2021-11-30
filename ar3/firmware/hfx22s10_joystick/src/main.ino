#define X_AXIS A7
#define Y_AXIS A6
#define JOY_BUTTON A5
#define TOUCH_BUTTON1 A4
#define TOUCH_BUTTON2 A3
#define TOUCH_BUTTON3 A2

void setup() {
  Serial.begin(115200);

  pinMode(X_AXIS, INPUT);
  pinMode(Y_AXIS, INPUT);
  pinMode(JOY_BUTTON, INPUT_PULLUP);
  pinMode(TOUCH_BUTTON1, INPUT);
  pinMode(TOUCH_BUTTON2, INPUT);
  pinMode(TOUCH_BUTTON3, INPUT);
  
}

char buf[32];
void loop() {
  uint16_t x = analogRead(X_AXIS);
  uint16_t y = analogRead(Y_AXIS);
  byte b[] = {
    !digitalRead(JOY_BUTTON), // Active low
    digitalRead(TOUCH_BUTTON1),
    digitalRead(TOUCH_BUTTON2),
    digitalRead(TOUCH_BUTTON3)
  };
  sprintf(buf, "%d %d %d %d %d %d", x, y, b[0], b[1], b[2], b[3]);
  Serial.println(buf);
  delay(50);
}
