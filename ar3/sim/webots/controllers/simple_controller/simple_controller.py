"""simple_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, PositionSensor

NUM_JOINTS = 6
DANCE_TIME = 1.0

robot = Robot()
motors = [robot.getMotor("joint_%d" % i) for i in range(1,NUM_JOINTS)]
sensors = [m.getPositionSensor() for m in motors]
timestep = int(robot.getBasicTimeStep())

for s in sensors:
    s.enable(timestep)

ti = 0
targets = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
    [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
]


lastdance = 0
while robot.step(timestep) != -1:
    now = robot.getTime()
    if now - lastdance > DANCE_TIME:
        lastdance = now
        for i, m in enumerate(motors):
            m.setPosition(targets[ti][i])
        print("New target:", targets[ti])
        ti = (ti + 1) % len(targets)
        
    # print([s.getValue() for s in sensors])
