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
from geometry_msgs.msg import Vector3, TransformStamped
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
import rclpy
import math

class Pendant(WebotsNode):

    def __init__(self, args):
        super().__init__('pendant_controller', args)
        self.get_logger().info("Init")
        self.pub = self.create_publisher(TFMessage, "tf", 10)
        #self.sub = self.create_subscription(Pose, "cmd", self.set_pose, qos_profile=qos_profile_sensor_data)
        self.sub = self.create_subscription(Vector3, "vr/pos", self.set_pos, qos_profile=qos_profile_sensor_data)
        self.timer = self.create_timer(0.01 * self.timestep, self.publish_tf)
        self.tfield = self.robot.getSelf().getField("translation")
        self.get_logger().info("Ready")

    def set_pos(self, v3):
        self.tfield.setSFVec3f([v3.x, v3.z, v3.y]) # ROS is Z up
        # TODO rotation https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Recovering_the_axis-angle_representation
        # self.robot.rotation = pose.orientation

    def publish_tf(self):
        # modded from https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_core/webots_ros2_core/tf_publisher.py
	# Publish TF for the next step
        # we use one step in advance to make sure no sensor data are published before
        nextTime = self.robot.getTime() + 0.001 * self.timestep
        nextSec = int(nextTime)
        # rounding prevents precision issues that can cause problems with ROS timers
        nextNanosec = int(round(1000 * (nextTime - nextSec)) * 1.0e+6)
        node = self.robot.getSelf()
        if node is None:
            print("No self node")
            return
        position = node.getPosition()
        orientation = node.getOrientation()
        transformStamped = TransformStamped()
        transformStamped.header.stamp = Time(sec=nextSec, nanosec=nextNanosec)
        transformStamped.header.frame_id = 'map'
        transformStamped.child_frame_id = self.robot.getName()
        transformStamped.transform.translation.x = position[0]
        transformStamped.transform.translation.y = position[2] # ROS is Z-up
        transformStamped.transform.translation.z = position[1]
        qw = math.sqrt(1.0 + orientation[0] + orientation[4] + orientation[8]) / 2.0
        qx = (orientation[7] - orientation[5]) / (4.0 * qw)
        qy = (orientation[2] - orientation[6]) / (4.0 * qw)
        qz = (orientation[3] - orientation[1]) / (4.0 * qw)
        transformStamped.transform.rotation.x = qx
        transformStamped.transform.rotation.y = qy
        transformStamped.transform.rotation.z = qz
        transformStamped.transform.rotation.w = qw
        self.pub.publish(TFMessage(transforms=[transformStamped]))

def main(args=None):
    rclpy.init(args=args)
    exampleController = Pendant(args=args)
    rclpy.spin(exampleController)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
