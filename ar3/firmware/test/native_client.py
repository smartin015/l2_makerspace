import zmq
import sys
import struct
import threading
import argparse
import websockets
import asyncio
from collections import defaultdict
from http.server import HTTPServer, BaseHTTPRequestHandler

NUM_J = 6
mask = [0]*NUM_J
pos = [0]*NUM_J
vel = [0]*NUM_J
STRUCT_FMT = 'B'*NUM_J + 'h'*NUM_J + 'h'*NUM_J


class WebServer(BaseHTTPRequestHandler):
  def do_GET(self):
    with open('./index.html', 'r') as f:
      self.send_response(200)
      self.send_header("Content-type", "text/html")
      self.end_headers()
      self.wfile.write(bytes(f.read(), 'utf8'))

async def handle_socket(ws, path):
  global mask, pos, vel, args
  while True:
    data = await ws.recv()
    mpv = data.split("|")
    mask = [int(v) for v in mpv[0].split(",")]
    pos = [int(v) for v in mpv[1].split(",")]
    vel = [int(v) for v in mpv[2].split(",")]
    req = struct.pack(STRUCT_FMT, *(mask + pos + vel))
    print(req.hex(),"--->",end='')
    if args.loopback:
      await asyncio.sleep(0.1)
      resp = req
    else: 
      socket.send(req)
      resp = socket.recv()
    print(resp.hex()) # print(mask,pos,vel,"-->",end='')
    mpv = struct.unpack(STRUCT_FMT, resp)
    mask = mpv[0:NUM_J]
    pos = mpv[NUM_J:2*NUM_J]
    vel = mpv[2*NUM_J:]
    # print(mask,pos,vel)
    await ws.send("|".join([",".join([str(s) for s in v]) for v in [mask, pos, vel]]))

parser = argparse.ArgumentParser()
parser.add_argument('--loopback', action='store_true', default=False, help="Transmit to self, for testing")
args = parser.parse_args(sys.argv[1:])

context = zmq.Context()


#  Socket to talk to server
print("Connecting...")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

if __name__ == "__main__":
  srv = HTTPServer(("0.0.0.0", 8000), WebServer)
  threading.Thread(target=srv.serve_forever, daemon=True).start()
  print("Started server on port 8000")

  wssrv = websockets.serve(handle_socket, "0.0.0.0", 8001)
  asyncio.get_event_loop().run_until_complete(wssrv)
  print("Starting ws event loop (port 8001)")
  asyncio.get_event_loop().run_forever()
