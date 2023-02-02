import serial
from time import sleep

with serial.Serial(port='/dev/cu.usbmodem2101', baudrate=9600) as arduino:
    while not arduino.isOpen():
        print('waiting to open')
        arduino.open()

    ready = arduino.read()
    if ready != b'r':
        exit(-1)

    print("arduino is ready")

    sleep(0.1)
    arduino.write(b'95')
    sleep(0.1)
    print(arduino.read())
    print(arduino.read())

    print(arduino.read_all())

    # print(arduino.readline())

    # print('sending bytes: ', arduino.write(b'77'))
    # sleep(2)

    # arduino.write(b'09')
    # sleep(5)''
