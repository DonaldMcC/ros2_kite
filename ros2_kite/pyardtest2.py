# Importing Libraries
# This proves two way comms between arduino and python - arduino adds 1 to the number you send it
# will require a sketch on arduino that does the increment
# possibly look at pySerialTransfer and SerialTransfer.h
#  - this looks simple https://linuxhint.com/serial-read-serial-write-function-arduino/
# might be buffering to consider and read rather than readline may work better
# not sure I can do a lot more with this until got board back

import serial
from time import sleep

board = serial.Serial(port='COM5', baudrate=115200, timeout=.1)


def write_read(x):
    board.write(bytes(x, 'utf-8'))
    sleep(0.05)
    data = board.readline()
    return data


while True:
    num = input("Enter a number: ")  # Taking input from user
    value = write_read(num)
    print(value)  # printing the value
