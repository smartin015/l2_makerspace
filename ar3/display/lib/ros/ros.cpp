#include "ros_hal.h"
#include "ros.h"

namespace {

bool ledState = false;

void subscribeLed(ros_hal::std_msgs::Bool* msg, void* arg) {
  ledState = msg->data;
}

} // namespace

void ros_setup(const char* server_ip, const char* server_port) {
  // ros2::init(client);
  ros_hal::init(server_ip, server_port);
  ros_hal::sub("arduino_led", "bool", ros_hal::CallbackFunc(subscribeLed));
}

void ros_loop() {
  ros_hal::spin();
}
