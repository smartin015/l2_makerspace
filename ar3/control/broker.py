import os
import signal
import sys
import struct
import threading
import argparse
import websockets
import asyncio
from random import random
from collections import defaultdict
from http.server import HTTPServer, SimpleHTTPRequestHandler

try:
    import zmq
except:
    pass # ZMQ not needed as a dependency on the robot itself

PACKET_START_BYTE = 0x02 # Matches ../firmware/hal/micro/comms.cpp
NUM_J = None # Overridden by args
MAX_BUFFERED_MESSAGES = 10

class Broker():
    def __init__(self, dest, pull=None, motion=None):
        self.motion = motion
        self.sock = None
        self.push = None
        self.pull = None
        self.q = asyncio.Queue()
        if dest.startswith('/dev'):
            print("Opening serial connection:", dest)
            import serial
            self.sock = serial.Serial(port=dest, baudrate=115200, timeout=0.1)
        else: # Network
            #  Socket to talk to server
            self.ctx = zmq.Context()
            print("Connecting to fw comms PUSH socket:", dest)
            self.push = self.ctx.socket(zmq.PUSH)
            self.push.connect(dest)
            print("Connecting to fw comms PULL socket:", pull)
            self.pull = self.ctx.socket(zmq.PULL)
            self.pull.connect(pull)

    def read_forever(self):
        print("Starting comms read loop")
        if self.pull is not None:
            while True:
                try:
                    data = self.pull.recv()
                except zmq.error.Again:
                    continue
                self.q.put_nowait(data)
        else:
            while True:
                stuff = self.sock.read_until(bytes([PACKET_START_BYTE]))
                if stuff == b'':
                    print("No serial data")
                    continue
                sz = self.sock.read(1)
                if len(sz) != 1:
                    print("ERR serial could not read size after sync byte; runup:", stuff[-10:])
                    continue
                sz = int(sz[0])

                # Bound the queue size, but without throwing QueueFull exceptions as the usual implementation goes
                while self.q.qsize() > MAX_BUFFERED_MESSAGES:
                    self.q.get_nowait()

                if sz == PACKET_START_BYTE: # Print status messages to console
                    s = '[FW] ' + self.sock.read_until(bytes([0])).decode('utf-8').rstrip('\x00\n\r')
                    print(s)
                    self.q.put_nowait(s)
                    continue
                else:
                    # Note: we don't do any length checking - this is the responsibility of the consumer
                    self.q.put_nowait(self.sock.read(sz))
                

    async def recv_forever(self, ws):
        while True:
            msg = await self.q.get()
            await ws.send(msg)

    def send(self, req):
        if self.sock is not None:
            self.sock.write(bytes([PACKET_START_BYTE, len(req)]))
            self.sock.write(req)
            self.sock.flush()
        else:
            self.push.send(req)

conn = None

def sigterm_handler(_signo, _stack_frame):
    sys.exit(0)
signal.signal(signal.SIGTERM, sigterm_handler)

class WebServer(SimpleHTTPRequestHandler):
    def do_GET(self):
        global NUM_J
        if self.path == "/config":
            self.send_response(200)
            self.send_header("Content-type", "text/plain")
            self.end_headers()
            self.wfile.write(bytes(str(NUM_J), "utf-8"))
        else:
            return SimpleHTTPRequestHandler.do_GET(self)

async def handle_socket(ws, path):
  global NUM_J
  print("WS conn", str(ws))
  if args.loopback:
      await ws_to_conn(ws)
  else:
      await asyncio.gather(ws_to_conn(ws), conn.recv_forever(ws))

async def ws_to_conn(ws):
  print("Starting ws_to_conn")
  async for req in ws:
    #print(req.hex(),"--->")
    if args.loopback:
      await asyncio.sleep(0.1)
      await ws.send(req)
      if random() < 0.05:
        await ws.send('[FW] test loopback message')
    else:
      conn.send(req)

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--motion', default="passthrough", help="Type of motion control to use")
  parser.add_argument('--loopback', action='store_true', default=False, help="Transmit to self, for testing")
  parser.add_argument('-j', default=6, type=int, help="Number of joints in robot (affects packet size & controls display")
  parser.add_argument('--dest', default="tcp://0.0.0.0:5559", help="Destination (ZMQ socket or serial dev path)")
  parser.add_argument('--pull', default="tcp://0.0.0.0:5558", help="PULL socket destination (ignored when --dest is serial")
  parser.add_argument('--websocket_port', default=8001, help="Port for websocket control")
  parser.add_argument('--http_port', default=8000, help="Port for HTTP server")
  parser.add_argument('--web_dir', default="www", help="Web dir (relative to .py script)")

  args = parser.parse_args(sys.argv[1:])
  NUM_J = args.j

  web_dir = os.path.join(os.path.dirname(__file__), args.web_dir)
  print("Serving files from", web_dir)
  os.chdir(web_dir)

  # Webserver for html page
  WEB_SERVER_ADDR = ("0.0.0.0", args.http_port)
  srv = HTTPServer(WEB_SERVER_ADDR, WebServer)
  threading.Thread(target=srv.serve_forever, daemon=True).start()
  print("Started web server", str(WEB_SERVER_ADDR))

  # Websocket for streaming comms from web client
  WS_SERVER_ADDR = ("0.0.0.0", args.websocket_port)
  wssrv = websockets.serve(handle_socket, WS_SERVER_ADDR[0], WS_SERVER_ADDR[1])

  if not args.loopback:
      conn = Broker(args.dest, args.pull, args.motion)
      threading.Thread(target=conn.read_forever, daemon=True).start()

  asyncio.get_event_loop().run_until_complete(wssrv)
  print("Starting websocket server", str(WS_SERVER_ADDR))
  asyncio.get_event_loop().run_forever()
