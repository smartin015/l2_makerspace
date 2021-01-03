# from l2_msgs.srv import GetProject
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from controller import Robot, Motor, PositionSensor
from std_msgs.msg import Float64, Header
from trajectory_msgs.msg import JointTrajectory
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import threading

MOTOR_LIMITS = [
  (-2.4, 2.4),
  (-2.4, 1.8),
  (-4.5, 1.3),
  (-2.4, 2.4),
  (-2.4, 2.4),
  (-3.14, 3.14),
]

# Normalized for minposition-maxposition -1.0 to 1.0
TARGETS = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.1, 0.0, 0.0, 0.0],
    [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0],
    [0.2, 0.4, 0.6, 0.8, 1.0, 1.0],
    [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
]

class AR3(Node):
    PUBLISH_PD = 5  # seconds
    JOINT_STATE_PD = 0.100
    DANCE_PD = 5.0
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
        self.joint_names = ["joint_%d" % (i+1) for i in range(self.NUM_JOINTS)]
        self.motors = [self.robot.getMotor(n) for n in self.joint_names]
        for (i,m) in enumerate(self.motors):
            # https://cyberbotics.com/doc/reference/motor?tab-language=python#motor
            pid = (1.0, 0, 0)
            m.setControlPID(*pid)
            m.minPosition = MOTOR_LIMITS[i][0]
            m.maxPosition = MOTOR_LIMITS[i][1]
            print("Motor %d configured for PID %s, range (%f,%f)" % (i, pid, m.getMinPosition(), m.getMaxPosition()))
        self.sensors = [m.getPositionSensor() for m in self.motors]
        for s in self.sensors:
            s.enable(self.timestep)

        # Setup topics & timers
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self._default_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.ti = 0
        self.stbc = StaticTransformBroadcaster(self)
        self.tbc = TransformBroadcaster
        self.pub = self.create_publisher(Float64, 'sense', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.last_joint_state_ts = self.get_clock().now()
        self.last_joint_state = [0 for i in self.joint_names]

        # For some reason (lots of clock publishes?) the ros timer implementation causes a bug
        # where one timer is dependent on the execution speed of the other timer.
        # We use a manual implementation in spin_sim() instead.
        # self.joint_state_timer = self.create_timer(self.JOINT_STATE_PD, self.joint_state_callback)
        # self.dance_timer = self.create_timer(self.DANCE_PD, self.dance_callback)
        self.command_sub = self.create_subscription(JointTrajectory, "joint_trajectory_command", self.handle_joint_trajectory, 
                                                    qos_profile=qos_profile_sensor_data)
        self.get_logger().info("Init")

    def spin_sim(self):
        next_joint_state_cb = 0
        next_dance_cb = 0
        while self.robot.step(self.timestep) != -1:
            now = self.robot.getTime()
            self.clockpub.publish(Clock(clock=Time(sec=int(now), nanosec=int((now % 1) * 1000000000))))
            if now > next_joint_state_cb:
              self.joint_state_callback()
              next_joint_state_cb = next_joint_state_cb + self.JOINT_STATE_PD
            if now > next_dance_cb:
              self.dance_callback()
              next_dance_cb = next_dance_cb + self.DANCE_PD

    def handle_joint_trajectory(self, jt):
        # print(jt) # TODO handle setting joint trajectory
        for (i, name) in enumerate(jt.joint_names):
            # TODO - catch exceptions
            mi = int(name[-1]) - 1 # Convert to index into motors
            self.motors[mi].setPosition(jt.points[0].positions[i])

    def joint_state_callback(self):
        pos = [s.getValue() for s in self.sensors]
        now = self.get_clock().now()
        dt = float((now - self.last_joint_state_ts).nanoseconds) / (1000000000)
        if dt < 0.00001:
            vel = [0 for n in self.joint_names]
        else:
            vel=[(a-b)/dt for (a,b) in zip(pos, self.last_joint_state)]

        self.joint_state_pub.publish(JointState(
            header=Header(stamp=now.to_msg()),
            name=self.joint_names,
            position=pos,
            velocity=vel,
            effort=[0 for n in self.joint_names],
        ))
        self.last_joint_state_ts = now
        self.last_joint_state = pos

    def dance_callback(self):
        self.ti = (self.ti + 1) % len(TARGETS)
        for i, m in enumerate(self.motors):
            m.setPosition((TARGETS[self.ti][i] + 1.0)/2.0 * (MOTOR_LIMITS[i][1]-MOTOR_LIMITS[i][0]) + MOTOR_LIMITS[i][0])
        self.get_logger().info("New target " + str(TARGETS[self.ti]))

def main(args=None):
    rclpy.init(args=args)
    server = AR3()
    threading.Thread(target=server.spin_sim, name='sim', daemon=True).start()
    rclpy.spin(server, executor=server.executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
