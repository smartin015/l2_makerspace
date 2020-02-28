import os
import rclpy
from rclpy.node import Node

class RLServer(Node):
    PUBLISH_PD = 60  # seconds

    def __init__(self):
        super().__init__('l2_rl_server')
        self.get_logger().info("Init")

def main(args=None):
    rclpy.init(args=args)
    server = RLServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
