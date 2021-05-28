# Requires sudo apt-get install xboxdrv
# and pip install -U xbox360controller
import time
import math
import serial
import websockets
from xbox360controller import Xbox360Controller

PI=3.14159

class Controller:
    NUM_J = 3

    def __init__(self, controller, ws, max_speed, publish_pd, loopback):
        self.controller = controller
        self.ws = ws
        self.mask = [0]*Controller.NUM_J
        self.pos = None
        self.vel = None
        self.max_speed = max_speed
        self.publish_pd = publish_pd
        self.loopback = loopback
        self.struct_fmt = '=' + 'B'*self.NUM_J + 'i'*self.NUM_J + 'h'*self.NUM_J
        print("Expected packet format:", struct_fmt)
        controller.button_a.when_pressed = self.on_button_pressed
        controller.button_a.when_released = self.on_button_released

    def on_button_pressed(self, button):
        print('TODO button {0} was pressed'.format(button.name))

    def on_button_released(self, button):
        print('TODO button {0} was released'.format(button.name))

    def wheel_speed(self, vx, vy, theta):
        if vx == 0 and vy == 0:
          return 0
        # Max 1.0 magnitude to prevent going faster than max_speed on a diagonal (which could be up to 1.414)
        mag = min(math.sqrt(vx*vx + vy*vy), 1.0)
        ang = math.atan2(vy, vx) 
        return mag * math.cos(ang - theta)

    def wheel_cmd(self, vx, vy, rx, ry):
        if vx == 0 and vy == 0 and rx == 0:
          return None

        # Forward is -Y +Z
        # Left is +X, 

        spds = [self.wheel_speed(vx, vy, theta) for theta in [0, 2*PI/3, 4*PI/3]]

        # Turning is added on top of existing speed
        spds = [s + rx for s in spds]

        # Multiply by feed rate
        # return ("G91 G1 X%04f Y%04f Z%04f" % tuple([s * self.max_speed/1000 for s in spds])) + (" F%d" % FEED)
        return struct.pack(self.struct_fmt, *(self.mask + self.pos + self.vel))


    def debug_directions(self):
        # Simple check that the wheel magnitudes are reasonable
        print(self.wheel_cmd(0, 0, 0, 0))
        print(self.wheel_cmd(1, 0, 0, 0))
        print(self.wheel_cmd(-1, 0, 0, 0))
        print(self.wheel_cmd(0, 1, 0, 0))
        print(self.wheel_cmd(0, -1, 0, 0))

    async def send_commands(self):
        while True:
            await asyncio.sleep(self.publish_pd)
            if self.pos is None:
                continue # Wait until we know where we are before we decide where to go
            cmd = self.wheel_cmd(
                    self.controller.axis_l.x, 
                    self.controller.axis_l.y, 
                    self.controller.axis_r.x, 
                    self.controller.axis_r.y)
            if cmd is not None:
                await self.ws.send(cmd)
                print(cmd)

    async def receive_state(self):
        async for msg in self.ws:
            try:
                mpv = struct.unpack(self.struct_fmt, msg)
            except Exception as e:
                print("Error unpacking msg of size %d:" % len(msg), e)
            # Don't just naively read mask; this would cause e.g. limit switch states to be ignored
            # state.mask = mpv[0:NUM_J]
            self.pos = mpv[NUM_J:2*NUM_J]
            self.vel = mpv[2*NUM_J:]
            print(self.pos)

    async def spin(self):
        done, pending = await asyncio.wait(
            [asyncio.ensure_future(self.receive_state()),
             asyncio.ensure_future(self.send_commands())],
            return_when=asyncio.FIRST_COMPLETED,
        )
        for task in pending:
            task.cancel()

class LoopbackServer():
    def __init__(self):
        self.wss = websockets.serve(self.echo, "localhost", 8765)

    async def echo(self, ws, path):
        async for msg in ws:
            await ws.send(msg)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--loopback', action='store_true', default=False, help="Transmit to self, for testing")
    parser.add_argument('--dest', default="ws://192.168.1.250:8001", help="Destination websocket broker server; ignored if --loopback is set")
    parser.add_argument('--max_speed', default=400, help="Maximum speed (steps/second)")
    parser.add_argument('--publish_pd', default=0.1, help="Period in seconds to publish controller values")
    args = parser.parse_args(sys.argv[1:])
    try:
        with Xbox360Controller(0) as controller:
            if args.loopback:
                wss = LoopbackServer()
                dest = wss.dest
            else:
                dest = args.dest
            with websockets.connect(args.dest) as ws:
            con = Controller(ws, controller, args.max_speed, args.publish_pd, args.loopback)
            await con.spin()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
