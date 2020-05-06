# from l2_msgs.srv import GetProject
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, CompressedImage
import os
import rclpy
import struct
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

GD_VARIANT = "ii%dB"
GD_RAW_ARRAY = 20
UDP_DEST = ("vr", 4242)
import socket
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

class TFFwd(Node):
    PUBLISH_PD = 0.1  # seconds
    SNAPSHOT_PD = 10.0 # seconds

    def __init__(self, ns="/l2"):
        super().__init__('l2_vr_fwd', namespace=ns)
        self.get_logger().info("Init")
        cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.rvl_sub = self.create_subscription(CompressedImage, 'rvl', self.rvl_callback,
                                                qos_profile=qos_profile_sensor_data,
                                                callback_group=cb_group)
        self.rvl_count = 0
        self.sub = self.create_subscription(JointState, 'joint_states', 
                self.sub_callback, qos_profile=qos_profile_sensor_data,
                callback_group=cb_group)

        self.pub = self.create_publisher(JointState, 'vr/joint_states', 10)
                # qos_profile=qos_profile_sensor_data)
        self.joint_meta = {} # name: [last_published, similarity_score]

    def rvl_callback(self, msg):
        if msg.format != "RVL":
            self.get_logger().error("Not forwarding image format %s" % msg.format, throttle_duration_sec=5.0)
            return

        try:
            sock.sendto(struct.pack(GD_VARIANT % len(msg.data), GD_RAW_ARRAY, len(msg.data), *msg.data), UDP_DEST)
            self.rvl_count += 1
        except socket.gaierror:
            return
        self.get_logger().error("Sent %d rvl messages since start" % self.rvl_count, throttle_duration_sec=10.0)

    def sub_callback(self, joint_states):
        msg = JointState(header=joint_states.header)
        ts = joint_states.header.stamp
        for i in range(len(joint_states.name)):
            name = joint_states.name[i]
            p = joint_states.position[i]
            v = joint_states.velocity[i]
            e = joint_states.effort[i]
            score = p+v+e
            (prev_ts, prev_score) = self.joint_meta.get(name, (None, None))
            if prev_ts is None or (prev_ts-ts > Duration(seconds=0.5) and abs(score-prev_score) > 0.01):
                msg.name.append(name)
                msg.position.append(p)
                msg.velocity.append(v)
                msg.effort.append(e)
        if len(msg.name) == 0:
            return
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    server = TFFwd()
    try:
        rclpy.spin(server)
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
