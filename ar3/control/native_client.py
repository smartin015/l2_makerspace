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

NUM_J = 6
mask = [0]*NUM_J
pos = [0]*NUM_J
vel = [0]*NUM_J
STRUCT_FMT = 'B'*NUM_J + 'h'*NUM_J + 'h'*NUM_J

def sigterm_handler(_signo, _stack_frame):
    sys.exit(0)
signal.signal(signal.SIGTERM, sigterm_handler)

class WebServer(SimpleHTTPRequestHandler):
    pass

async def handle_socket(ws, path):
  global mask, pos, vel, args
  print("WS conn", str(ws))
  while True:
    # NOTE: Expected units are in "steps", "steps/second" etc.
    data = await ws.recv()
    mpv = data.split("|")
    mask = [int(v) for v in mpv[0].split(",")]
    pos = [int(v) for v in mpv[1].split(",")]
    vel = [int(v) for v in mpv[2].split(",")]
    req = struct.pack(STRUCT_FMT, *(mask + pos + vel))
    # print(req.hex(),"--->",end='')
    if args.loopback:
      await asyncio.sleep(0.1)
      resp = req
    else: 
      socket.send(req)
      resp = socket.recv()
    # print(resp.hex()) # print(mask,pos,vel,"-->",end='')
    mpv = struct.unpack(STRUCT_FMT, resp)
    mask = mpv[0:NUM_J]
    pos = mpv[NUM_J:2*NUM_J]
    vel = mpv[2*NUM_J:]
    #print(mask,pos,vel)
    await ws.send("|".join([",".join([str(s) for s in v]) for v in [mask, pos, vel]]))

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--loopback', action='store_true', default=False, help="Transmit to self, for testing")
  args = parser.parse_args(sys.argv[1:])

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
