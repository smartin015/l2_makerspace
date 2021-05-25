#ifndef _HW_H
#define _HW_H

#include <stdint.h>

#define HIGH true
#define LOW false

namespace hw {
  bool get_cur_cal(int idx);
  int get_steps(int idx);

  void move_steps(int idx, int delta);
  
  void sync(); 
  void init();
  void loop();

  uint64_t millis();

} // namespace hw

#endif
