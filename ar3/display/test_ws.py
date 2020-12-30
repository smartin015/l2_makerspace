#!/usr/bin/env python
import asyncio
import websockets
import time
import json

async def handler(websocket, path):
    print("Handler starting")
    while True:
        await asyncio.sleep(2)
        print("sending msg")
        await websocket.send(json.dumps({
            "pos":    [0, 10, 20, 30, 40, 50],
            "target": [10, 20, 30, 40, 50, 60],
            "herp": "hi",
        }))

start_server = websockets.serve(handler, "localhost", 8000)
asyncio.get_event_loop().run_until_complete(start_server)
print("Started server, looping")
asyncio.get_event_loop().run_forever()
