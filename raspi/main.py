import threading
import cv2, serial, numpy as np

import asyncio
import websockets
from io import BytesIO
from time import sleep

# Configuration
HEADLESS = True
TRACKER = "csrt"
SERIAL_PORT = '/dev/cu.usbmodem2101'
BAUDRATE = 9600
WEBSOCKET_PORT = 8764
FRAME_DIMENSIONS = (1280, 720)

# Communication with Arduino
arduino = None
arduino_state = None

# Interface between server and tracker
ACTIVE_FRAME = None
LAST_SEND_FRAME = None
TO_TRACK = None
STATE = "IDLE"


async def handler(websocket):
    global STATE, LAST_SEND_FRAME, TO_TRACK, ACTIVE_FRAME
    async for message in websocket:
        print(message)

        if message == "c":
            print("Companion is connected")
            await websocket.send("ok")
        elif message == "f":
            print("Frame requested")
            LAST_SEND_FRAME = ACTIVE_FRAME

            # Compress frame
            ok, compressed = cv2.imencode('.jpg', LAST_SEND_FRAME)

            # Save to Byte Stream
            np_bytes = BytesIO()
            np.save(np_bytes, compressed, allow_pickle=True)

            await websocket.send(np_bytes.getvalue())
        elif message == "t":
            print("Track region received")
            ROI = eval(await websocket.recv())
            TO_TRACK = (LAST_SEND_FRAME, ROI)
            STATE = "TRACK"
        elif message == "a":
            connect_to_arduino()


async def start_server_internal():
    async with websockets.serve(handler, "localhost", WEBSOCKET_PORT, ping_timeout=None):
        await asyncio.Future()  # run forever


def start_server():
    asyncio.run(start_server_internal())

def connect_to_arduino():
    arduino = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE)
    arduino.__enter__()

    while not arduino.isOpen():
        print('waiting to open')
        arduino.open()
        sleep(0.1)

    sleep(0.1)

    ready = arduino.read()
    if ready != b'r' or arduino is None:
        arduino_state = "failed"
        arduino.__exit__()
        arduino = None
    else:
        arduino_state = "connected"


wssTask = threading.Thread(target=start_server)
wssTask.start()

print("Server started")

OPENCV_OBJECT_TRACKERS = {
    "csrt": cv2.TrackerCSRT_create,
    "kcf": cv2.TrackerKCF_create,
    "mil": cv2.TrackerMIL_create,
}

# Set up video capture
video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_DIMENSIONS[0])
video.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_DIMENSIONS[1])
if not video:
    print("[ERROR] Failed VideoCapture: invalid parameter!")
    exit(1)

# Main Loop
while True:
    # Capture frame-by-frame
    ret, frame = video.read()
    if type(frame) == type(None):
        print("[ERROR] Couldn't read frame!")
        break

    f_height = len(frame)
    f_width = len(frame[0])
    f_center = (int(f_width / 2), int(f_height / 2))

    # Start FPS timer
    timer = cv2.getTickCount()

    if not HEADLESS:
        keyPress = cv2.waitKey(1) & 0xFF

        # Press A to connect to Arduino
        if keyPress == ord('a'):
            connect_to_arduino()

        # Press T to define Tracking Region
        if keyPress == ord('t'):
            STATE = "TRACK"

            bbox = cv2.selectROI(frame, False)
            tracker = OPENCV_OBJECT_TRACKERS[TRACKER]()

            # Initialize tracker with first frame and bounding box
            ok = tracker.init(frame, bbox)

    if STATE == "TRACK":
        if TO_TRACK is not None:
            # Initialized Tracker
            tracker = OPENCV_OBJECT_TRACKERS[TRACKER]()
            track_frame, bbox = TO_TRACK
            ok = tracker.init(frame, bbox)  # Initialize tracker with first frame and bounding box
            TO_TRACK = None

        # Update tracker
        ok, bbox = tracker.update(frame)

        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            p_center = (int((p1[0] + p2[0]) / 2), int((p1[1] + p2[1]) / 2))

            # Draw bounding box
            cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)

            # Draw distance from center
            cv2.line(frame, f_center, p_center, (255, 0, 0), 2, 1)

            vector = (
                p_center[0] - f_center[0],
                p_center[1] - f_center[1]
            )
            normalized_vector = (
                vector[0] / (f_width / 2),
                vector[1] / (f_height / 2)
            )
            clamped_vector = (
                min(max(-1, normalized_vector[0]), 0.99),
                min(max(-1, normalized_vector[1]), 0.99),
            )
            scaled_vector = (
                int(clamped_vector[0] * 5) + 5,
                int(clamped_vector[1] * 5) + 5
            )

            cv2.putText(frame, "Center offset: " + str(clamped_vector), (100, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (255, 255, 0), 2)

            cv2.putText(frame, "to arduino: " + str(scaled_vector), (100, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (255, 255, 0), 2)

            if arduino is not None:
                message = f'{scaled_vector[0]}{scaled_vector[1]}'.encode('UTF-8')
                arduino.write(message)
                print(arduino.read(), arduino.read_all())
            # Tracking success
        else:
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    # Display tracker type on frame
    cv2.putText(frame, TRACKER + " Tracker", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

    # Display tracker type on frame
    cv2.putText(frame, str(f_width) + "x" + str(f_height), (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

    # Calculate and Display FPS
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

    cv2.putText(frame, "Arduino " + "not connected - Press A to connect" if arduino_state is None else arduino_state, (100, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                (255, 255, 0), 2)

    # Make frame accessible to server
    ACTIVE_FRAME = frame

    # Display the resulting frame
    if not HEADLESS:
        cv2.imshow('frame', frame)

        if keyPress == ord('q'):
            break

# release the capture
video.release()
cv2.destroyAllWindows()

# close arduino
if arduino is not None:
    arduino.__exit__()

exit(0)
