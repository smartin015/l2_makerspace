#include "command.h"

const char ENC_CHAR[] = {'A', 'B', 'C', 'D', 'E', 'F', 'T'};
const char EXTRA_CHAR[] = {'G', 'H', 'I', 'K', 'S', '\0', '\0'};
const char J_CHAR[] = {'U', 'V', 'W', 'X', 'Y', 'Z', '\0'};

command_t parse_command(String inData) {
  command_t result;
  result.function[0] = inData[0];
  result.function[1] = inData[1];

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
  return result;
}

