# Requires sudo apt-get install xboxdrv
# and pip install -U xbox360controller
import asyncio
import struct
import time
import math
import serial
import websockets
import argparse
import sys
import logging
from xbox360controller import Xbox360Controller

PI=3.14159

# see ../firmware/src/state.h
MASK_ENABLED_OPEN_LOOP = 0b101
MASK_DISABLED = 0b000

class Controller:
    NUM_J = 3

    def __init__(self, controller, ws, max_speed, publish_pd, stats_pd, loopback):
        self.controller = controller
        self.ws = ws
        self.mask = [MASK_ENABLED_OPEN_LOOP]*Controller.NUM_J
        self.pos = None
        self.vel = None
        self.max_speed = max_speed
        self.publish_pd = publish_pd
        self.loopback = loopback
        self.stats_pd = stats_pd
        self.stats = {"pub": 0}
        self.struct_fmt = '=' + 'B'*self.NUM_J + 'i'*self.NUM_J + 'h'*self.NUM_J
        logging.info(f"Expected packet format: {self.struct_fmt}")
        controller.button_a.when_pressed = self.on_button_pressed
        controller.button_a.when_released = self.on_button_released

    def on_button_pressed(self, button):
        logging.warning(f'TODO button {button.name} pressed')

    def on_button_released(self, button):
        logging.warning(f'TODO button {button.name} released')

    def wheel_speed(self, vx, vy, theta):
        if vx == 0 and vy == 0:
          return 0
        # Max 1.0 magnitude to prevent going faster than max_speed on a diagonal (which could be up to 1.414)
        mag = min(math.sqrt(vx*vx + vy*vy), 1.0)
        ang = math.atan2(vy, vx) 
        return mag * math.cos(ang - theta)

    def wheel_cmd(self, vx, vy, rx, ry):
        if self.pos is None or self.vel is None:
          return None

        if vx == 0 and vy == 0 and rx == 0:
          spds = [0] * self.NUM_J
          self.mask = [MASK_ENABLED_OPEN_LOOP] * self.NUM_J
        else:
          # Forward is -Y +Z
          # Left is +X, 
          spds = [self.wheel_speed(vx, vy, theta) for theta in [0, 2*PI/3, 4*PI/3]]

          # Turning is added on top of existing speed
          spds = [int(self.max_speed * (s + rx)) for s in spds]
          # Update wheel enabled state
          # self.mask = [MASK_ENABLED_OPEN_LOOP if s != 0 else MASK_DISABLED for s in spds] 
        

        # Update position
        pos = [int(self.pos[i] + spds[i]*self.publish_pd) for i in range(self.NUM_J)]
        self.pos = pos
        logging.info(f"pos {pos} speeds {spds}")
        spds = [int(0) for i in range(self.NUM_J)]
        return struct.pack(self.struct_fmt, *(self.mask + pos + spds))


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
                self.stats['pub'] = self.stats['pub'] + 1
                await self.ws.send(cmd)
                # print(cmd)

    async def receive_state(self):
        async for msg in self.ws:
            if type(msg) == str:
              logging.info(msg)
              continue
            try:
                mpv = struct.unpack(self.struct_fmt, msg)
            except Exception as e:
                logging.error(f"Error unpacking msg of size {len(msg)}: {str(e)}")
                continue
            # Don't just naively read mask; this would cause e.g. limit switch states to be ignored
            # state.mask = mpv[0:NUM_J]
            npos = list(mpv[self.NUM_J:2*self.NUM_J])
            if self.pos is None:
              self.pos = npos
            # self.pos = [npos[i] if abs(npos[i]-self.pos[i]) > 10 else self.pos for i in range(self.NUM_J)]
            self.vel = list(mpv[2*self.NUM_J:])
            # print(self.pos)

    async def periodic_stats(self):
        while True:
            await asyncio.sleep(self.stats_pd)
            logging.info(f"stats: {self.stats['pub'] / self.stats_pd} publishes/sec")
            self.stats['pub'] = 0

    async def spin(self):
        done, pending = await asyncio.wait(
            [asyncio.ensure_future(self.receive_state()),
             asyncio.ensure_future(self.send_commands()),
             asyncio.ensure_future(self.periodic_stats())],
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

async def run(dest, controller, args):
    async with websockets.connect(dest) as ws:
        con = Controller(controller, ws, args.max_speed, args.publish_pd, args.stats_pd, args.loopback)
        await con.spin()


def main():
    logging.basicConfig(format="%(asctime)s %(levelname)s: %(message)s", level=logging.INFO)
    parser = argparse.ArgumentParser()
    parser.add_argument('--loopback', action='store_true', default=False, help="Transmit to self, for testing")
    parser.add_argument('--dest', default="ws://192.168.1.250:8001", help="Destination websocket broker server; ignored if --loopback is set")
    parser.add_argument('--max_speed', default=800, type=int, help="Maximum speed (steps/second)")
    parser.add_argument('--publish_pd', default=0.08, type=float, help="Period in seconds to publish controller values")
    parser.add_argument('--stats_pd', default=5.0, type=float, help="Stats logging period")
    args = parser.parse_args(sys.argv[1:])
    try:
        with Xbox360Controller(0) as controller:
            if args.loopback:
                wss = LoopbackServer()
                dest = wss.dest
            else:
                dest = args.dest
            logging.info("Destination websocket:", dest)
            asyncio.get_event_loop().run_until_complete(run(dest, controller, args))
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
