# Importing Libraries
import turtle
import serial
from serial import SerialException

connected = False
import time

try:
    arduino = serial.Serial(port='/dev/cu.usbmodem101', baudrate=115200, timeout=.1)
    connected = True
except SerialException:
    print("arduino not connected")
    connected = False

s = turtle.getscreen()
t = turtle.Turtle()
# t.hideturtle()
t.speed("fastest")


class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i + 1]
            self.buf = self.buf[i + 1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i + 1]
                self.buf[0:] = data[i + 1:]
                return r
            else:
                self.buf.extend(data)


# ser = serial.Serial('COM7', 9600)
rl = ReadLine(arduino)

while True:
    start_time = time.time()

    value = ["0", "0"]

    if connected:
        # value = arduino.readline()
        value = rl.readline()

        # print(value) # printing the value

        if value == "":
            continue

        if value[0] == "c":
            t.clear()
            continue

        value = value.split(" ")

    # print(value)

    for i in range(0, len(value), 2):
        try:
            x = int(value[i].strip())
            y = int(value[i+1].strip())
        except:
            x = 0
            y = 0
        t.penup()
        t.goto(x, y)
        t.down()
        t.dot(4)

    print("--- %s seconds ---" % (time.time() - start_time))

