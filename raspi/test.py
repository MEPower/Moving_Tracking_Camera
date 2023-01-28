import serial

with serial.Serial(port='/dev/tty.usbmodem2101', baudrate=9600) as arduino:
    while not arduino.isOpen():
        print('waiting to open')
        arduino.open()

    print('sending bytes: ', arduino.write(b'77'))

    while True:
        print(arduino.read())
