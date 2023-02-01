#!/usr/bin/env python3

import asyncio
import websockets
import cv2
import numpy as np
from io import BytesIO

# Configuration
SERVER = "localhost:8764" # IP:PORT
SERVER_TYPE = "C++"  # C++ or Python


async def client():
    async with websockets.connect("ws://" + SERVER, max_size=None, read_limit=2 ** 20) as websocket:
        print("Raspberry Connected, press f to get frame, press t to select tracking area")

        # Handshake
        await websocket.send("c")
        success = await websocket.recv()

        while True:
            keypress = cv2.waitKey(1) & 0xFF
            await asyncio.sleep(0.1)

            # Request frame
            await websocket.send("f")
            serialized = await websocket.recv()

            # receive encoded frame (jpg)
            if SERVER_TYPE == "C++":
                loaded_bytes = bytearray(serialized)
                compressed = np.asarray(loaded_bytes, dtype="ubyte")
            else:
                loaded_bytes = BytesIO(serialized)
                compressed = np.load(loaded_bytes)

            frame = cv2.imdecode(compressed, cv2.IMREAD_UNCHANGED)

            # Alter and show frame
            cv2.putText(frame, "Press Q to close", (100, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 200, 100), 2)
            cv2.putText(frame, "Press T to track", (100, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 200, 100), 2)
            cv2.imshow('companion', frame)

            if keypress == ord('t'):
                cv2.destroyWindow('companion')
                bbox = cv2.selectROI('select', frame, False)
                cv2.imshow('companion', frame)
                cv2.destroyWindow('select')

                # Request rpi to track image region in bbox
                await websocket.send("t")
                await websocket.send(str(bbox))
            elif keypress == ord('a'):
                await websocket.send("a")
            elif keypress == ord('q'):
                cv2.destroyAllWindows()
                break


asyncio.run(client())
