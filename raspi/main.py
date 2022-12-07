import cv2, platform, sys, serial, numpy as np

OPENCV_OBJECT_TRACKERS = {
    "csrt": cv2.TrackerCSRT_create,
    "kcf": cv2.TrackerKCF_create,
    "mil": cv2.TrackerMIL_create,
}

# Tracking Algorithm
TRACKER = "csrt"

# Communication with Arduino
BAUDRATE = 115200
arduino = None

# State of this script
state = ""

# Set up video capture
video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
if not video:
    print("!!! Failed VideoCapture: invalid parameter!")
    exit(1)

# Main Loop
while (True):
    
    # Capture frame-by-frame
    ret, frame = video.read()
    if type(frame) == type(None):
        print("!!! Couldn't read frame!")
        break

    f_height = len(frame)
    f_width = len(frame[0])
    f_center = (int(f_width / 2), int(f_height / 2))

    # Start FPS timer
    timer = cv2.getTickCount()
    
    
    # Press C to connect to Arduino
    if cv2.waitKey(1) & 0xFF == ord('c'):
        arduino = serial.Serial(port='COM4', baudrate=BAUDRATE, timeout=.1)

    # Press T to define Tracking Region
    if cv2.waitKey(1) & 0xFF == ord('t'):
        state = "TRACK"

        bbox = cv2.selectROI(frame, False)
        tracker = OPENCV_OBJECT_TRACKERS[TRACKER]()

        # Initialize tracker with first frame and bounding box
        ok = tracker.init(frame, bbox)
    
    if state == "TRACK":
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
                min(max(-1, normalized_vector[0]), 1),
                min(max(-1, normalized_vector[1]), 1),
            )
            scaled_vector = (
                int(clamped_vector[0] * 255), 
                int(clamped_vector[1] * 255)
            )

            cv2.putText(frame, "Correction vector" + str(clamped_vector), (100, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (255, 255, 0), 2)

            cv2.putText(frame, str(scaled_vector), (100, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (255, 255, 0), 2)

            if arduino is not None:
                # TO BE DONE
                arduino.write(str(scaled_vector))

        else:
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    # Display tracker type on frame
    cv2.putText(frame, TRACKER + " Tracker", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

    # Display tracker type on frame
    cv2.putText(frame, str(f_width) + "x" + str(f_height), (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

    # Calculate and Display FPS
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# release the capture
video.release()
cv2.destroyAllWindows()

# # Importing Libraries
# import serial
# import time
# arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)
# def write_read(x):
#     arduino.write(bytes(x, 'utf-8'))
#     time.sleep(0.05)
#     data = arduino.readline()
#     return data
# while True:
#     num = input("Enter a number: ") # Taking input from user
#     value = write_read(num)
#     print(value) # printing the value

# Arduino Code
# int x;
#
# void setup() {
#   Serial.begin(115200);
#   Serial.setTimeout(1);
# }
#
# void loop() {
#   while (!Serial.available());
#   x = Serial.readString().toInt();
#   Serial.print(x + 1);
# }
