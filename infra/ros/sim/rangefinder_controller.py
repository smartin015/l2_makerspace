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
from geometry_msgs.msg import Pose, TransformStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
import rclpy
import math

class Rangefinder(WebotsNode):
    def __init__(self, args):
        super().__init__('pendant_controller', args)
        self.get_logger().info("Init")
        self.timestep = int(self.robot.getBasicTimeStep())
        self.pub = self.create_publisher(TFMessage, "tf", 10)
        self.range = self.robot.getRangeFinder("rangefinder")
        self.range.enable(self.timestep)
        self.timer = self.create_timer(0.001 * self.timestep, self.publish_rvl)
        self.get_logger().info("Ready")
        self.lastUpdate = -100

    def publish_rvl(self):
        if self.robot.getTime() - self.lastUpdate < (self.range.getSamplingPeriod() / 1000.0):
            return
        self.robot.step(self.timestep)
        img = self.range.getRangeImage()
        msg = Image(height=self.range.getHeight(), width=self.range.getWidth(), encoding="RVL")
        
        # TODO convert image array to RVL and send over UDP
        self.lastUpdate = self.robot.getTime()

def main(args=None):
    rclpy.init(args=args)
    exampleController = Rangefinder(args=args)
    rclpy.spin(exampleController)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
