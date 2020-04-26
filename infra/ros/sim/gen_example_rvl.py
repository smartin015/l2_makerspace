# 25 = PoolVector3Array type
# 4 4 Integer Array Length
# 8..8+length*12  4 Float X Coordinate
# 8..12+length*12 4 Float Y Coordinate
# 8..16+length*12 4 Float Z Coordinate
import math
import socket
import struct
import time
import sys
from os import path
from rvl import rvl

# from l2_msgs.srv import GetProject
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

GODOT_RAW_ARRAY_TYPE = 20

import socket
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
UDP_DEST = ("vr", 4242)

class GenRVL(Node):
    SCALE_FACTOR = 1000
    FMT = "ii%dB"
    PUBLISH_PD = 0.1  # seconds
    FRAME_ID = "range"

    def __init__(self):
        super().__init__('l2_example')
        self.get_logger().info("Init")
        #self.pub = self.create_publisher(CompressedImage, 'rvl', 10)
        self.timer = self.create_timer(self.PUBLISH_PD, self.gen_and_send)

    def gen_and_send(self):
        ts = time.time()
        data = []
        w = 128
        nvecs = w*w
        for i in range(w):
            for j in range(w):
                v = (math.sin(i/16.0 + ts/3.0) + math.cos(j/16.0 + ts/3.0)) * 0.1
                data += [int(v*self.SCALE_FACTOR)]
        rvl.Clear()
        rvl.plain = data
        rvl.CompressRVL(chan=0)
        packed = struct.pack(self.FMT % len(rvl.encoded), GODOT_RAW_ARRAY_TYPE, len(rvl.encoded), *rvl.encoded)
        #msg = CompressedImage(header=Header(frame_id=self.FRAME_ID, stamp=self.get_clock().now().to_msg()), format="RVL", data=packed)
        #self.pub.publish(msg)
        sock.sendto(packed, UDP_DEST)


def main(args=None):
    rclpy.init(args=args)
    server = GenRVL()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
