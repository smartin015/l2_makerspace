#ifndef CONFIG_H
#define CONFIG_H

#define NUM_J 6

// MOTOR DIRECTION - motor directions can be changed on the caibration page in the software but can also
// be changed here: example using DM542T driver(CW) set to 1 - if using ST6600 or DM320T driver(CCW) set to 0
// DEFAULT = 1110110
const int ROT_DIR[] = {0,0,0,0,0,0,0};

// start positions - these are the joint step values at power up
// default is in the rest position using the following values: 
// J1=7600, J2=2322, J3=0, J4=7600, J5=2287, J6=3312
const int START_POS[] = {7600, 2322, 0, 7600, 2287, 3312, 0};

const int STEP_PIN[] = {0, 2, 4, 6, 8, 10};
const int DIR_PIN[]  = {1, 3, 5, 7, 9, 11};

//set encoder pins
// TODO move to app_hal
#ifndef NATIVE

#include <Encoder.h>
Encoder enc[] = {
	Encoder(14, 15),
	Encoder(16, 17),
	Encoder(18, 19),
	Encoder(20, 21),
	Encoder(22, 23),
	Encoder(24, 25)
};

#endif

//set calibration limit switch pins
const int CAL_PIN[] = {26, 27, 28, 29, 30, 31};

//set encoder multiplier
//const double ENC_MULT[] = {5.12, 5.12, 5.12, 5.12, 2.56, 5.12};
//const float ENC_DIV = 0.1;

#endif // CONFIG_H
