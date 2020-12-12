# from l2_msgs.srv import GetProject
import os
import rclpy
from rclpy.node import Node
from controller import Robot, Motor, PositionSensor
from std_msgs.msg import Float64

targets = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
]
class AR3(Node):
    PUBLISH_PD = 60  # seconds

    def __init__(self):
        super().__init__('l2_ar3')
        self.get_logger().info("Init")
        self.robot = Robot()
        self.motors = [self.robot.getMotor("joint_%d" % i) for i in range(1, NUM_JOINTS)]
        self.sensors = [m.getPositionSensor() for m in self.motors]
        self.timestep = int(robot.getBasicTimeStep())

        for s in self.sensors:
            s.enable(self.timestep)

        self.ti = 0
        self.last = 0
        # TODO publish /clock http://wiki.ros.org/Clock#Using_Simulation_Time_from_the_.2Fclock_Topic
        self.pub = self.create_publisher(Float64, 'sense', 10)
        self.timer = self.create_timer(self.PUBLISH_PD, self.timer_callback)

    def step(self):
        rclpy.spin_once(self)
        now = self.robot.getTime()
        if now - self.last > 1.0:
            self.last = now
            self.ti = (self.ti + 1) % len(targets)
            for i, m in enumerate(self.motors):
                m.setPosition(targets[self.ti][i])
            print("New target ", targets[self.ti])
        return self.robot.step(self.timestep)

    # def timer_callback(self):
        # self.get_logger().info("TODO")
        # pass #todo self.pub.publish()

def main(args=None):
    rclpy.init(args=args)
    server = AR3()
    while server.step():
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
