import zmq
import signal
import sys
import struct
import threading
import argparse
import websockets
import asyncio
from collections import defaultdict
from http.server import HTTPServer, SimpleHTTPRequestHandler

NUM_J = None # Overridden by args

class State():
    def __init__(self):
        global NUM_J
        self.struct_fmt = 'B'*NUM_J + 'h'*NUM_J + 'h'*NUM_J
        self.mask = [0]*NUM_J
        self.pos = [0]*NUM_J
        self.vel = [0]*NUM_J

state = None

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
  global state, NUM_J
  print("WS conn", str(ws))
  while True:
    # NOTE: Expected units are in "steps", "steps/second" etc.
    data = await ws.recv()
    mpv = data.split("|")
    state.mask = [int(v) for v in mpv[0].split(",")]
    state.pos = [int(v) for v in mpv[1].split(",")]
    state.vel = [int(v) for v in mpv[2].split(",")]
    req = struct.pack(state.struct_fmt, *(state.mask + state.pos + state.vel))
    # print(req.hex(),"--->",end='')
    if args.loopback:
      await asyncio.sleep(0.1)
      resp = req
    else: 
      socket.send(req)
      resp = socket.recv()
    # print(resp.hex()) # print(state.mask,state.pos,state.vel,"-->",end='')
    mpv = struct.unpack(state.struct_fmt, resp)
    state.mask = mpv[0:NUM_J]
    state.pos = mpv[NUM_J:2*NUM_J]
    state.vel = mpv[2*NUM_J:]
    #print(state.mask,state.pos,state.vel)
    await ws.send("|".join([",".join([str(s) for s in v]) for v in [state.mask, state.pos, state.vel]]))

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--loopback', action='store_true', default=False, help="Transmit to self, for testing")
  parser.add_argument('-j', default=6, type=int, help="Number of joints in robot (affects packet size & controls display")
  args = parser.parse_args(sys.argv[1:])
  NUM_J = args.j
  state = State()

  #  Socket to talk to server
  ZMQ_SOCK_ADDR = "tcp://0.0.0.0:5555"
  context = zmq.Context()
  print("Opening ZMQ REQ socket on", ZMQ_SOCK_ADDR)
  socket = context.socket(zmq.REQ)
  socket.connect(ZMQ_SOCK_ADDR)

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
