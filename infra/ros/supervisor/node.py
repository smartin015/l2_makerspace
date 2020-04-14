# from l2_msgs.srv import GetProject
import os
import rclpy
import docker # https://docker-py.readthedocs.io/en/stable/
import yaml
from rclpy.node import Node

class Supervisor(Node):
    PUBLISH_PD = 5  # seconds

    def __init__(self):
        super().__init__('l2_example')
        self.get_logger().info("Init")
        #self.pub = self.create_publisher(ProjectsUpdate, 'project', 10)
        self.timer = self.create_timer(self.PUBLISH_PD, self.timer_callback)
        self.client = docker.from_env()
        self.config = {}
        self.load_config()

    def load_config(self):
        path = "/config.yaml"
        self.get_logger().info("Loading from %s" % path)
        with open(path, 'r') as stream:
            try:
                self.config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                self.get_logger().error(exc)
        self.get_logger().info(str(self.config))

    def timer_callback(self):
        self.get_logger().info(str(self.client.containers.list()))
        pass #todo self.pub.publish()

def main(args=None):
    rclpy.init(args=args)
    server = Supervisor()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
