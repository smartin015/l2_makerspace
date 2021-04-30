# from l2_msgs.srv import GetProject
import os
import glob
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
import zmq
import sys
import signal
import struct
import threading
import time

def sigterm_handler(_signo, _stack_frame):
    print("SIGTERM received, exiting")
    sys.exit(0)
signal.signal(signal.SIGTERM, sigterm_handler)

# NOTE: If num_joints < 6, the first num_joints values are used
MOTOR_LIMITS = [
  (-2.4, 2.4),
  (-2.4, 1.8),
  (-4.5, 1.3),
  (-2.4, 2.4),
  (-2.4, 2.4),
  (-3.14, 3.14),
]

# Normalized for minposition-maxposition -1.0 to 1.0
# NOTE: if num_joints < 6, the first num_joints values are used per target
TARGETS = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.1, 0.0, 0.0, 0.0],
    [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0],
    [0.2, 0.4, 0.6, 0.8, 1.0, 1.0],
    [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
]

# NOTE: If num_joints < 6, the first num_joints values are used
STEPS_PER_REV = [4096, 4096, 4096, 4096, 4096, 4096]

class StubRobot:
  def __init__(self):
    self.t = 0
    pass
  def getBasicTimeStep(self):
    return 0.01
  def getMotor(self, name):
    return StubMotor(name)
  def step(self, timestep):
    self.t += timestep
    return 0
  def getTime(self):
    return self.t


class StubPosSensor:
  def __init__(self):
    self.value = 0
  def enable(self, timestep):
    pass
  def getValue(self):
    return self.value

class StubMotor:
  def __init__(self, name):
    self.name = name
    self.minPosition = 0
    self.maxPosition = 0
    self.sensor = StubPosSensor()

  def getMinPosition(self):
    return self.minPosition
  def getMaxPosition(self):
    return self.maxPosition
  def setPosition(self, target):
    self.sensor.value = target
  def setControlPID(self, p, i, d):
    pass
  def getPositionSensor(self):
    return self.sensor
    

class AR3(Node):
    PUBLISH_PD = 5  # seconds
    JOINT_STATE_PD = 0.100
    DANCE_PD = 5.0

    def __init__(self):
        super().__init__('l2_ar3')
        self.declare_parameter("stub", "false")
        self.declare_parameter("step_url", "tcp://localhost:5556")
        self.declare_parameter("limit_url", "tcp://localhost:5557")
        self.declare_parameter("num_j", 6)
        self.num_joints = self.get_parameter('num_j').get_parameter_value().integer_value

        # Setup simulation using webots clock (and publishing for other ros2 nodes)
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', value=True)
        ])
        self.clockpub = self.create_publisher(Clock, '/clock', 10)

        self.joint_names = ["joint_%d" % (i+1) for i in range(self.num_joints)]

        if self.get_parameter('stub').get_parameter_value().bool_value:
          self.get_logger().info("Using StubRobot")
          self.robot = StubRobot()
        else:
          self.get_logger().info("Waiting for webots to initialize")
          webots_initialized = False
          while not webots_initialized:
            webots_initialized = len(glob.glob("/tmp/webots_*_*", recursive=False)) > 0
            time.sleep(1)
            self.get_logger().info(".")

          self.get_logger().info("Initializing robot")
          # Robot() blocks checking for connection to Webots - see "Webots doesn't seems to be ready yet" in
          # https://github.com/cyberbotics/webots/blob/f9c017f55084b2d2ccf6cad7c94c97bb9c3ebd2c/src/controller/c/robot.c
          self.robot = Robot()
          if not self.robot:
            print("Robot init failed")
            os.exit(1)
        
        self.timestep = int(self.robot.getBasicTimeStep())
        self.motors = [self.robot.getMotor(n) for n in self.joint_names]
        for (i,m) in enumerate(self.motors):
            # https://cyberbotics.com/doc/reference/motor?tab-language=python#motor
            pid = (10.0, 0, 0)
            m.setControlPID(*pid)
            m.minPosition = MOTOR_LIMITS[i][0]
            m.maxPosition = MOTOR_LIMITS[i][1]
            print("Motor %d configured for PID %s, range (%f,%f)" % (i, pid, m.getMinPosition(), m.getMaxPosition()))
          
        self.sensors = [m.getPositionSensor() for m in self.motors]
        for s in self.sensors:
            s.enable(self.timestep)

        self.get_logger().info("Socket init...")
        STEP_URL = self.get_parameter('step_url').get_parameter_value().string_value
        LIM_URL = self.get_parameter('limit_url').get_parameter_value().string_value
        self.zmq_context = zmq.Context()
        self.step_socket = self.zmq_context.socket(zmq.PULL)
        self.step_socket.connect(STEP_URL)
        self.limits = [False]*self.num_joints
        self.lim_socket = self.zmq_context.socket(zmq.PUSH)
        self.lim_socket.connect(LIM_URL)
        self.get_logger().info("Pulling steps from %s; pushing limits state to %s" % (STEP_URL, LIM_URL)) 

        self.get_logger().info("Topic / timer init...")
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
        self.get_logger().info("Init complete!")

    def spin_sim(self):
        next_joint_state_cb = 0
        next_dance_cb = 0
        while self.robot.step(self.timestep) != -1:
            now = self.robot.getTime()
            self.clockpub.publish(Clock(clock=Time(sec=int(now), nanosec=int((now % 1) * 1000000000))))
            if now > next_joint_state_cb:
              self.joint_state_callback()
              next_joint_state_cb = next_joint_state_cb + self.JOINT_STATE_PD
            #if now > next_dance_cb:
            #  self.dance_callback()
            #  next_dance_cb = next_dance_cb + self.DANCE_PD

            try:
              steps = self.step_socket.recv(zmq.DONTWAIT)
              if steps:
                self.handle_raw_steps(steps)
            except zmq.error.Again:
              pass

    def handle_raw_steps(self, steps):
        # When simulating firmware, we get raw step counts over ZMQ - these are 
        # referenced relative from simulation start pos
        self.get_logger().info("Firmware sim conn active", throttle_duration_sec=5)
        limits_updated = False
        for (i, s) in enumerate(struct.unpack('i'*self.num_joints, steps)):
            angle = (2*3.14159) * s / STEPS_PER_REV[i]
            self.motors[i].setPosition(angle)
            # Only one side has actual limit switches; the other is a "breaking point"
            inside_limits = (MOTOR_LIMITS[i][0] < angle)
            if angle > MOTOR_LIMITS[i][1]:
              # Regardless of if we're on a side with a limit switch, we should throw warnings if we exceed
              # the non-switched side of the machine
              self.get_logger().warn("non-switch limit %f exceeded for joint %d: angle is %f (not reported to fw)" % (MOTOR_LIMITS[i][1], i, angle), throttle_duration_sec=1)
            if self.limits[i] != inside_limits:
              limits_updated = True
              self.get_logger().info("Limit switch %d state change - %d <- %d -> %d" % (i, MOTOR_LIMITS[i][0], angle, MOTOR_LIMITS[i][1]))
            self.limits[i] = inside_limits

        if limits_updated:
            self.lim_socket.send(struct.pack('b'*self.num_joints, *self.limits))
            self.get_logger().info("Sent limit update to firmware sim %s" % self.limits)

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
