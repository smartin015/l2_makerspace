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

class Comms():
    def __init__(self, dest):
        self.ser = dest.startswith('/dev')
        if self.ser:
            print("Opening serial connection:", dest)
            import serial
            self.sock = serial.Serial(port=dest, baudrate=115200, timeout=0.1)
        else: # Network
            #  Socket to talk to server
            print("Opening ZMQ REQ socket:", dest)
            import zmq
            self.context = zmq.Context()
            self.sock = context.socket(zmq.REQ)
            self.sock.connect(ZMQ_SOCK_ADDR)

    def recv_ser(self):
        while True:
            if self.sock.in_waiting == 0:
                return None
            self.sock.read_until(bytes([0x79])) #magic sync byte
            sz = self.sock.read(1)
            if len(sz) != 1:
                print("ERR serial could not read size")
                return None
            sz = int(sz[0])
            if sz == 0x79: # Print status messages to console
                print('[FW]', self.sock.read_until(bytes([0])).decode('utf-8').rstrip('\x00\n\r'))
                continue

            if sz != 5*NUM_J: # 2 shorts, 1 bool per joint
                print("ERR serial data read size: want %d got %d", 5*NUM_J, int(sz))
                return None
            return self.sock.read(sz)

    async def recv_forever(self, cb):
        print("Starting recv_forever")
        while True:
            data = self.recv_ser()
            if data is not None:
                await cb(data)
            else:
                await asyncio.sleep(0)


    def send(self, req):
        if self.ser:
            self.sock.write(bytes([0x79, len(req)]))
            self.sock.write(req)
            self.sock.flush()
            return None
        else:
            socket.send(req)
            return socket.recv()

conn = None

class State():
    def __init__(self):
        global NUM_J
        # = indicates native order, standard size (native size may add padding)
        self.struct_fmt = '=' + 'B'*NUM_J + 'h'*NUM_J + 'h'*NUM_J
        print("Expected packet format:", self.struct_fmt)
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
  # Note: async read not threadsafe
  async def send_resp(resp):
    try:
        mpv = struct.unpack(state.struct_fmt, resp)
    except Exception as e:
        print("Error unpacking response of size %d:" % len(resp), e)
    state.mask = mpv[0:NUM_J]
    state.pos = mpv[NUM_J:2*NUM_J]
    state.vel = mpv[2*NUM_J:]
    #print(state.mask,state.pos,state.vel)
    await ws.send("|".join([",".join([str(s) for s in v]) for v in [state.mask, state.pos, state.vel]]))
  if args.loopback:
      await ws_to_conn(ws, send_resp)
  else:
      await asyncio.gather(conn.recv_forever(send_resp), ws_to_conn(ws))

async def ws_to_conn(ws, cb = None):
  print("Starting ws_to_conn")
  async for data in ws:
    mpv = data.split("|")
    state.mask = [int(v) for v in mpv[0].split(",")]
    state.pos = [int(v) for v in mpv[1].split(",")]
    state.vel = [int(v) for v in mpv[2].split(",")]
    req = struct.pack(state.struct_fmt, *(state.mask + state.pos + state.vel))
    # print(req.hex(),"--->")
    if args.loopback:
      await asyncio.sleep(0.1)
      await cb(req)
    else:
      conn.send(req)

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--loopback', action='store_true', default=False, help="Transmit to self, for testing")
  parser.add_argument('-j', default=6, type=int, help="Number of joints in robot (affects packet size & controls display")
  parser.add_argument('--dest', default="tcp://0.0.0.0:5555", help="Destination (ZMQ socket or serial dev path)")
  args = parser.parse_args(sys.argv[1:])
  NUM_J = args.j
  state = State()

  if not args.loopback:
      conn = Comms(args.dest)

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
