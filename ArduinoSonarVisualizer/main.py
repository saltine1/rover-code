# Importing Libraries
import turtle
import serial
from serial import SerialException

connected = False

try:
    arduino = serial.Serial(port='COM4', baudrate=9600, timeout=.1)
    connected = True
except SerialException:
    print("arduino not connected")
    connected = False

s = turtle.getscreen()
t = turtle.Turtle()
t.hideturtle()

while True:
    value = ["0", "0"]
    if connected:
        value = arduino.readline()
        print(value) # printing the value

        value = value.split(" ")

    t.penup()
    t.goto(int(value[0]), int(value[1]))
    t.down()
    t.dot(4 )


