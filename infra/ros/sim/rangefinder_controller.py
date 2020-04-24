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
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
from rvl import rvl
import rclpy
import math
import struct

class Rangefinder(WebotsNode):
    FRAME_ID = "range"
    MAX_RANGE = 10.0
    FMT = "ii%dB"

    def __init__(self, args):
        super().__init__('pendant_controller', args)
        self.get_logger().info("Init")
        self.timestep = int(self.robot.getBasicTimeStep())
        self.pub = self.create_publisher(CompressedImage, "rvl", 10)
        self.range = self.robot.getRangeFinder("rangefinder")
        self.range.enable(self.timestep)
        self.timer = self.create_timer(0.001 * self.timestep, self.publish_rvl)
        self.get_logger().info("Ready")
        self.lastUpdate = -100

    def publish_rvl(self):
        now = self.robot.getTime()
        if now - self.lastUpdate < (self.range.getSamplingPeriod() / 1000.0):
            return
        self.robot.step(self.timestep)
        rvl.Clear()
        # Remap to 16-bit integer
        raw = self.range.getRangeImage()
        if raw is None:
            self.get_logger().info("No range image", throttle_duration_sec=3)
            return
        rvl.plain = [int(65535 * px / self.MAX_RANGE) for px in raw]
        rvl.CompressRVL()
        packed = struct.pack(self.FMT % len(rvl.encoded), 20, len(raw), *rvl.encoded)
        header = Header(frame_id=self.FRAME_ID, stamp=Time(sec=int(now), nanosec=int((now-int(now)) * 1.0e+6)))
        msg = CompressedImage(header=header, format="RVL", data=packed)
        self.pub.publish(msg)
        self.lastUpdate = self.robot.getTime()

def main(args=None):
    rclpy.init(args=args)
    exampleController = Rangefinder(args=args)
    rclpy.spin(exampleController)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
