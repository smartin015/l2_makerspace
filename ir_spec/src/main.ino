
#define NLEDS 7

#define IR1650 8
#define IR1550 7
#define IR1450 6
#define IR1300 5
#define IR1200 4
#define IR0850 3
#define WHITE 2

#define SENSE A1

#define PULSE_MS 5

const int LEDS[] = {WHITE, IR0850, IR1200, IR1300, IR1450, IR1550, IR1650};

void setup() {
	Serial.begin(115200);
	pinMode(SENSE, INPUT);
	for (int i = 0; i < NLEDS; i++) {
		pinMode(LEDS[i], OUTPUT);
		digitalWrite(LEDS[i], HIGH);
	}
}

void loop() {
	for (int i = 0; i < NLEDS; i++) {
		digitalWrite(LEDS[i], LOW);
		delay(PULSE_MS);
		Serial.print(analogRead(SENSE));
		Serial.print(",");
		digitalWrite(LEDS[i], HIGH);
		delay(PULSE_MS);
	}
	Serial.println();
}
