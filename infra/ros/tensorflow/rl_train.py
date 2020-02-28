import os
import rclpy
from rclpy.node import Node

class RLTrainer(Node):
    PUBLISH_PD = 60  # seconds

    def __init__(self):
        super().__init__('l2_rl_trainer')
        self.get_logger().info("Init")
        # TODO train a basic RL model to predict next output of a sine
        # wave coming from a ROS topic.
        # http://inoryy.com/post/tensorflow2-deep-reinforcement-learning/


def main(args=None):
    rclpy.init(args=args)
    server = RLTrainer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
