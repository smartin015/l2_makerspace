#ifndef ROS_HAL_H
#define ROS_HAL_H


namespace ros_hal {

namespace std_msgs {
  typedef struct {
    bool data;
  } Bool;
}

typedef void(*CallbackFunc)(void* msg, void* arg);

void init(const char* server_ip, const char* server_port);
void sub(const char* topic, const char* type, CallbackFunc func);
void spin();

} // namespace ros_hal

#endif //ROS_HAL_H
