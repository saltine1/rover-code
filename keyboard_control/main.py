# Importing Libraries
import serial
from pynput import keyboard
from serial import SerialException

connected = False
import time

try:
    arduino = serial.Serial(port='/dev/cu.usbmodem21101', baudrate=115200, timeout=.1)
    connected = True
except SerialException:
    print("arduino not connected")
    connected = False

while True:
    start_time = time.time()

    # The event listener will be running in this block
    with keyboard.Events() as events:
        # Block at most one second
        event = events.get(0.1)
        if event is None:
            pass
        else:
            k = event.key
            try:
                arduino.write(str(k).encode('utf-8'))
            except (TypeError, NameError, AttributeError) as e:
                print(e)
                pass

            print(k)

    # print("--- %s seconds ---" % (time.time() - start_time))

