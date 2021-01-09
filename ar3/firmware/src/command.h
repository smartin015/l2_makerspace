#ifndef PARSE_H
#define PARSE_H

#include "config.h"
#include <string>

#define String std::string

enum {
  ACCEL_DURATION = 0,
  ACCEL_SPEED = 1,
  DECEL_DURATION = 2,
  DECEL_SPEED = 3,
  SPEED = 4,
} extra_values;

struct command_t {
  char function[2];
  int enc[NUM_J];
  int step[NUM_J];
  float extra[NUM_J];
};

command_t parse_command(String inData);

#endif // PARSE_H
