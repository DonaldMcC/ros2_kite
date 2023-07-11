# Importing Libraries
# This proves two way comms between arduino and python - arduino adds 1 to the number you send it
# will require a sketch on arduino to use pySerial or alternatively the firmata sketch if using pyfirmata

import serial
from time import sleep
import pyfirmata

use_firmata = False
if use_firmata:
    port = 'COM5'
    board = pyfirmata.Arduino(port)
    it = pyfirmata.util.Iterator(board)
    it.start()

    # Define pins

else:
    board = serial.Serial(port='COM5', baudrate=115200, timeout=.1)

def write_read(x):
    board.write(bytes(x, 'utf-8'))
    sleep(0.05)
    data = board.readline()
    return data


while True:
    num = input("Enter a number: ") # Taking input from user
    value = write_read(num)
    print(value) # printing the value