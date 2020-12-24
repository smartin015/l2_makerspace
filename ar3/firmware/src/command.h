#ifndef PARSE_H
#define PARSE_H

#import "config.h"

const char ENC_CHAR = {'A', 'B', 'C', 'D', 'E', 'F', 'T'};
enum {
  ACCEL_DURATION = 0;
  ACCEL_SPEED = 1;
  DECEL_DURATION = 2;
  DECEL_SPEED = 3;
  SPEED = 4;
} extra_values;
const char EXTRA_CHAR = {'G', 'H', 'I', 'K', 'S', '\0', '\0'};
const char J_CHAR = {'U', 'V', 'W', 'X', 'Y', 'Z', '\0'};

struct command_t {
  char function[2];
  int enc[NUM_J];
  int step[NUM_J];
  float extra[NUM_J];
};

command_t parse_command(String inData) {
  command_t result;
  result.function = inData.substring(0, 2);

  // Commands follow a <Letter, Value> format
  char buf[16];
  char b = 0;
  for (int i = 0; i <= inData.length(); i++) {
    if (isalpha(inData[i]) || i == inData.length()) {
      // Found next character or end of string; process the buffer
      buf[++b] = 0;
      int val = atoi(buf+1);
      for (int j = 0; j < NUM_J; j++) {
        if (buf[0] == J_CHAR[j]) {
	  result.step[j] = val;
	  break;
	} else if (buf[0] == ENC_CHAR[j]) {
	  result.enc[j] = val;
	  break;
	} else if (buf[0] == EXTRA_CHAR[j]) {
	  result.extra[j] = val;
	  break;
	}
      }
      b = 0;
    }

    if (i < inData.length()) {
      buf[b++] = inData[i];
    }
  }
}

#endif PARSE_H
