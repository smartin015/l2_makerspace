#ifndef ROS_HAL_H
#define ROS_HAL_H

#include "state_generated.h"

namespace ros_hal {

typedef void(*CallbackFunc)(void* msg, void* arg);
void init();
const char* get_status();
const State* get_state();
void spin();

} // namespace ros_hal

#endif //ROS_HAL_H
