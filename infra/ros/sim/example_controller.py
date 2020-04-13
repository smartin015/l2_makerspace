# Copyright 1996-2019 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS2 example controller."""

from webots_ros2_core.webots_node import WebotsNode

import rclpy
from sensor_msgs.msg import JointState

class ExampleController(WebotsNode):

    def __init__(self, args):
        super().__init__('example_controller', args)
        self.get_logger().info("Init")
        self.timer = self.create_timer(0.01 * self.timestep, self.pub_callback)
        self.motor = self.robot.getMotor('motor1')
        self.motor.setPosition(float('inf'))
        self.motor.setVelocity(3.14/4)
        
        # Reports NaN if not enabled
        self.motor.getPositionSensor().enable(100)
        self.jointStatePublisher = self.create_publisher(JointState, '/l2/vr/joint_states', 10)
        self.get_logger().info("Ready")

    def pub_callback(self):
        msg = JointState(
            name=["motor1"], 
            position=[self.motor.getPositionSensor().getValue()])
        self.jointStatePublisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    exampleController = ExampleController(args=args)
    rclpy.spin(exampleController)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
