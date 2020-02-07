// Run this with
// > pio run -e native
// > .pio/build/native/program

#ifndef ARDUINO_LOLIN_D32_PRO

#include <string>
#include <iostream>

void setup() {
  std::cout << "TODO setup" << std::endl;
}

bool loop() {
  std::cout << "TODO loop" << std::endl;
  return true;
}

int main(int argc, char** argv) {
  setup();
  while (loop()) {}
  //state->save(engine);
}

#endif // !ARDUINO_LOLIN_D32_PRO
