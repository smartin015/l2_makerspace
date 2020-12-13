# from l2_msgs.srv import GetProject
import os
import rclpy
from rclpy.node import Node
from controller import Robot, Motor, PositionSensor
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
import threading

targets = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
]

class AR3(Node):
    PUBLISH_PD = 5  # seconds
    NUM_JOINTS = 6

    def __init__(self):
        super().__init__('l2_ar3')
        # Set up simulation using webots clock (and publishing for other ros2 nodes)
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', value=True)
        ])
        self.clockpub = self.create_publisher(Clock, '/clock', 10)

        # Initialize robot
        self.motors = [self.robot.getMotor("joint_%d" % i) for i in range(1, self.NUM_JOINTS)]
        self.sensors = [m.getPositionSensor() for m in self.motors]
        for s in self.sensors:
            s.enable(self.timestep)

        self.pub = self.create_publisher(Float64, 'sense', 10)
        self.timer = self.create_timer(self.PUBLISH_PD, self.timer_callback)
        self.get_logger().info("Init")

    def spin_sim(self):
        self.ti = 0
        self.last = 0

        while self.robot.step(self.timestep) != -1:
            now = self.robot.getTime()
            self.clockpub.publish(Clock(clock=Time(sec=int(now), nanosec=0)))
            if now - self.last > 1.0:
                self.last = now
                self.ti = (self.ti + 1) % len(targets)
                for i, m in enumerate(self.motors):
                    m.setPosition(targets[self.ti][i])
                self.get_logger().info("New target " + str(targets[self.ti]))

    def timer_callback(self):
        self.get_logger().info("TODO")
        #todo self.pub.publish()

def main(args=None):
    rclpy.init(args=args)
    server = AR3()
    threading.Thread(target=server.spin_sim, name='sim', daemon=True).start()
    # server.spin_sim()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
