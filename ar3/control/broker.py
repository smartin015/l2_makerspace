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

NUM_J = None # Overridden by args

class Comms():
    def __init__(self, dest, pull=None):
        self.sock = None
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

    async def recv_forever(self, ws):
        print("Starting recv_forever")
        if self.pull is not None:
            while True:
                try:
                    data = self.pull.recv(zmq.NOBLOCK)
                except zmq.error.Again:
                    await asyncio.sleep(0)
                    continue
                await ws.send(data)
        else:
            while True:
                if self.sock.in_waiting == 0:
                    await asyncio.sleep(0)
                    continue
                self.sock.read_until(bytes([0x79])) #magic sync byte
                sz = self.sock.read(1)
                if len(sz) != 1:
                    print("ERR serial could not read size")
                    await asyncio.sleep(0)
                    continue
                sz = int(sz[0])
                if sz == 0x79: # Print status messages to console
                    s = '[FW]', self.sock.read_until(bytes([0])).decode('utf-8').rstrip('\x00\n\r')
                    print(s)
                    await ws.send(s)
                    continue

                else:
                    # Note: we don't do any length checking - this is the responsibility of the web interface
                    await ws.send(self.sock.read(sz))
                

    def send(self, req):
        if self.sock is not None:
            self.sock.write(bytes([0x79, len(req)]))
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
  parser.add_argument('--loopback', action='store_true', default=False, help="Transmit to self, for testing")
  parser.add_argument('-j', default=6, type=int, help="Number of joints in robot (affects packet size & controls display")
  parser.add_argument('--dest', default="tcp://0.0.0.0:5559", help="Destination (ZMQ socket or serial dev path)")
  parser.add_argument('--pull', default="tcp://0.0.0.0:5558", help="PULL socket destination (ignored when --dest is serial")
  parser.add_argument('--web_dir', default="www", help="Web dir (relative to .py script)")
  args = parser.parse_args(sys.argv[1:])
  NUM_J = args.j

  if not args.loopback:
      conn = Comms(args.dest, args.pull)

  web_dir = os.path.join(os.path.dirname(__file__), args.web_dir)
  print("Serving files from", web_dir)
  os.chdir(web_dir)

  # Webserver for html page
  WEB_SERVER_ADDR = ("0.0.0.0", 8000)
  srv = HTTPServer(WEB_SERVER_ADDR, WebServer)
  threading.Thread(target=srv.serve_forever, daemon=True).start()
  print("Started web server", str(WEB_SERVER_ADDR))

  # Websocket for streaming comms from web client
  WS_SERVER_ADDR = ("0.0.0.0", 8001)
  wssrv = websockets.serve(handle_socket, WS_SERVER_ADDR[0], WS_SERVER_ADDR[1])

  asyncio.get_event_loop().run_until_complete(wssrv)
  print("Starting websocket server", str(WS_SERVER_ADDR))
  asyncio.get_event_loop().run_forever()
