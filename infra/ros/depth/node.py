# from l2_msgs.srv import GetProject
import os
import rclpy
from rclpy.node import Node
from extra_file import hello

class DepthNode(Node):
    PUBLISH_PD = 60  # seconds

    def __init__(self):
        super().__init__('l2_example')
        self.get_logger().info("Init")
        #self.pub = self.create_publisher(ProjectsUpdate, 'project', 10)
        self.timer = self.create_timer(self.PUBLISH_PD, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("TODO")
        pass #todo self.pub.publish()

def main(args=None):
    rclpy.init(args=args)
    server = DepthNode()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
