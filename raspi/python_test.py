import threading
import cv2, serial, numpy as np
import os

import asyncio
import websockets
from io import BytesIO
from time import sleep

import time
import csv
import struct

#path = "C:Users\\Ella\\Ella-Kopie\\Studium\\Master\\Semester1\\Eingebettete Betriebssysteme\\Moving_Tracking_Camera\\raspi"
#with open(os.path.join(path, "performance_logs.txt"), 'w') as csvfile:
#   filewriter = csv.writer(csvfile)

def create_logfile():
    with open("performance_logs_python.txt", 'w') as csvfile:
        print("File created")

#   filewriter = csv.writer(csvfile)
#   print("started logging")


# Set up video capture


# Main Loop
i=0
create_logfile()
while True:
    # Capture frame-by-frame
    if(i==20): break

    start_time = time.time()*1000

    end_time = time.time() * 1000
    total_time = end_time - start_time

    with open("performance_logs_python.txt", 'a') as csvfile:
        #filewriter = csv.writer(csvfile)
        #filewriter.writerow(total_time)

        #this results in little-endina amount of bytes
        #csvfile.write(str(total_time))
        csvfile.write(str(i))
        csvfile.write("\n")
        #print("oughta write to file")

    i=i+1


# release the capture
#video.release()
#cv2.destroyAllWindows()

# close arduino


#end_time = time.time()*1000
#total_time = end_time- start_time

#with open("performance_logs.txt", 'wb') as csvfile:
#    filewriter = csv.writer(csvfile)
#    filewriter.writerow(total_time)

exit(0)