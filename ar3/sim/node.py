# from l2_msgs.srv import GetProject
import os
import rclpy
from rclpy.node import Node
from controller import Robot, Motor, PositionSensor
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
import threading

targets = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
]

class AR3(Node):
    PUBLISH_PD = 5  # seconds
    JOINT_STATE_PD = 0.100
    DANCE_PD = 1.0
    NUM_JOINTS = 6

    def __init__(self):
        super().__init__('l2_ar3')
        # Setup simulation using webots clock (and publishing for other ros2 nodes)
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', value=True)
        ])
        self.clockpub = self.create_publisher(Clock, '/clock', 10)

        # Setup robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.joint_names = ["joint_%d" % i for i in range(1, self.NUM_JOINTS)]
        self.motors = [self.robot.getMotor(n) for n in self.joint_names]
        self.sensors = [m.getPositionSensor() for m in self.motors]
        for s in self.sensors:
            s.enable(self.timestep)

        # Setup topics & timers
        self.ti = 0
        self.pub = self.create_publisher(Float64, 'sense', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_state', 10)
        # self.timer = self.create_timer(self.PUBLISH_PD, self.timer_callback)
        self.last_joint_state_ts = self.get_clock().now()
        self.last_joint_state = [0 for i in self.joint_names]
        self.joint_state_timer = self.create_timer(self.JOINT_STATE_PD, self.joint_state_callback)
        # self.dance_timer = self.create_timer(self.DANCE_PD, self.dance_callback)
        self.get_logger().info("Init")

    def spin_sim(self):
        while self.robot.step(self.timestep) != -1:
            now = self.robot.getTime()
            self.clockpub.publish(Clock(clock=Time(sec=int(now), nanosec=int((now % 1) * 1000000000))))

    def joint_state_callback(self):
        pos = [s.getValue() for s in self.sensors]
        now = self.get_clock().now()
        dt = float((now - self.last_joint_state_ts).nanoseconds) / (1000000000)
        if dt < 0.00001:
            vel = [0 for n in self.joint_names]
        else:
            vel=[(a-b)/dt for (a,b) in zip(pos, self.last_joint_state)]

        self.joint_state_pub.publish(JointState(
            name=self.joint_names,
            position=pos,
            velocity=vel,
            effort=[0 for n in self.joint_names],
        ))
        self.last_joint_state_ts = now
        self.last_joint_state = pos

    def dance_callback(self):
        self.ti = (self.ti + 1) % len(targets)
        for i, m in enumerate(self.motors):
            m.setPosition(targets[self.ti][i])
        self.get_logger().info("New target " + str(targets[self.ti]))

def main(args=None):
    rclpy.init(args=args)
    server = AR3()
    threading.Thread(target=server.spin_sim, name='sim', daemon=True).start()
    # server.spin_sim()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
