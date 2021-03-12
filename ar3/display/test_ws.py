#!/usr/bin/env python

# Websocket test data sender to display
# This periodically changes joint targets and
# linearly sweeps the position indicator to them. 
# When the indicator reaches the target, the limit indicator turns on

import asyncio
import websockets
import time
import json
from random import random

NUM_J = 6
TARGET_PD = 3.0

async def handler(websocket, path):
    print("Handler starting")
    p = [0] * NUM_J
    last_target = 0
    while True:
        await asyncio.sleep(0.05)
        print("sending msg")
        now = time.time()

        if now > last_target + TARGET_PD:
            t = [int(random() * 360) for i in range(NUM_J)]
            last_target = now
        for i in range(NUM_J):
            d = t[i] - p[i]
            if d != 0:
                p[i] = int(p[i] + (d / abs(d)) * min(3, abs(d)))
        await websocket.send(json.dumps({
            "pos": p,
            "target": t,
            "limit": [p[i] == t[i] for i in range(NUM_J)],
        }))

start_server = websockets.serve(handler, "0.0.0.0", 8001)
asyncio.get_event_loop().run_until_complete(start_server)
print("Started server, looping")
asyncio.get_event_loop().run_forever()
