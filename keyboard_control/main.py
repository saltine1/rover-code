# Importing Libraries
import serial
import keyboard
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

    k = keyboard.read_key()

    try:
        serial.Serial.write(k)
        print(k)
    except TypeError:
        pass

    print("--- %s seconds ---" % (time.time() - start_time))

