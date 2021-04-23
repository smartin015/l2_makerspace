# Requires sudo apt-get install xboxdrv
# and pip install -U xbox360controller
import time
import math
import serial
import socket
from xbox360controller import Xbox360Controller

FEED = 400
PI=3.14159

sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
dest = ("192.168.1.200", 20000)

def on_button_pressed(button):
    print('Button {0} was pressed'.format(button.name))


def on_button_released(button):
    print('Button {0} was released'.format(button.name))


def wheel_speed(vx, vy, theta):
    if vx == 0 and vy == 0:
      return 0
    mag = math.sqrt(vx*vx + vy*vy)
    ang = math.atan2(vy, vx) 
    return mag * math.cos(ang - theta)

def wheel_gcode(vx, vy, rx, ry):
    if vx == 0 and vy == 0 and rx == 0:
      return None

    # Forward is -Y +Z
    # Left is +X, 

    spds = [wheel_speed(vx, vy, theta) for theta in [0, 2*PI/3, 4*PI/3]]

    # Turning is added on top of existing speed
    spds = [s + rx for s in spds]

    # Multiply by feed rate
    return ("G1 X%04f Y%04f Z%04f" % tuple([s * FEED/100 for s in spds])) + (" F%d" % FEED)

def test_directions():
    # Simple check that the wheel magnitudes are reasonable
    print(wheel_gcode(0, 0, 0, 0))
    print(wheel_gcode(1, 0, 0, 0))
    print(wheel_gcode(-1, 0, 0, 0))
    print(wheel_gcode(0, 1, 0, 0))
    print(wheel_gcode(0, -1, 0, 0))
    import sys
    sys.exit(0)


try:
    with Xbox360Controller(0, axis_threshold=0.2) as controller:
        # Button A events
        controller.button_a.when_pressed = on_button_pressed
        controller.button_a.when_released = on_button_released

        while True:
          time.sleep(0.1)
          gcode = wheel_gcode(controller.axis_l.x, controller.axis_l.y, controller.axis_r.x, controller.axis_r.y)
          if gcode is not None:
            sock.sendto(gcode.encode("utf-8"), dest)
            print(gcode)

except KeyboardInterrupt:
    pass


