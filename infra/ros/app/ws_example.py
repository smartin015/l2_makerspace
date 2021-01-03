#!/usr/bin/env python

# WS client example

import asyncio
import websockets

async def hello():
    uri = "ws://0.0.0.0:8001"
    async with websockets.connect(uri) as websocket:
        await websocket.send("JOINT_STATE")
        while True:
          print(await websocket.recv())

asyncio.get_event_loop().run_until_complete(hello())
