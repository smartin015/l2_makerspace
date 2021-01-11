#ifndef _HW_H
#define _HW_H

#include "config.h"

namespace hw {
  bool get_cur_cal(int idx);
  bool get_steps(int idx);

  bool move_target(int idx, int delta);
  
  void sync(); 
  void init();
  void loop();
} // namespace hw


#endif
