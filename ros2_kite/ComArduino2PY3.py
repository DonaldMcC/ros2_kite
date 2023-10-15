# 19 July 2014
# 08 Dec 2016 - updated for Python3

# in case any of this upsets Python purists it has been converted from an equivalent JRuby program
# this is designed to work with ... ArduinoPC2.ino ...

# the purpose of this program and the associated Arduino program is to demonstrate a system for sending 
#   and receiving data between a PC and an Arduino.

# The key functions are:
#    sendToArduino(str) which sends the given string to the Arduino. The string may 
#                       contain characters with any of the values 0 to 255
#
#    recvFromArduino()  which returns an array. 
#                         The first element contains the number of bytes that the Arduino said it included in
#                             message. This can be used to check that the full message was received.
#                         The second element contains the message as a string

# the overall process followed by the demo program is as follows
#   open the serial connection to the Arduino - which causes the Arduino to reset
#   wait for a message from the Arduino to give it time to reset
#   loop through a series of test messages
#      send a message and display it on the PC screen
#      wait for a reply and display it on the PC

# to facilitate debugging the Arduino code this program interprets any message from the Arduino
#    with the message length set to 0 as a debug message which is displayed on the PC screen

# the message to be sent to the Arduino starts with < and ends with >
#    the message content comprises a string, an integer and a float
#    the numbers are sent as their ascii equivalents
#    for example <LED1,200,0.2>
#    this means set the flash interval for LED1 to 200 millisecs
#      and move the servo to 20% of its range

# receiving a message from the Arduino involves
#    waiting until the startMarker is detected
#    saving all subsequent bytes until the end marker is detected

# NOTES
#       this program does not include any timeouts to deal with delays in communication
#
#       for simplicity the program does NOT search for the comm port - the user must modify the
#         code to include the correct reference.
#         search for the lines 
#               serPort = "/dev/ttyS80"
#               baudRate = 9600
#               ser = serial.Serial(serPort, baudRate)
# So appears this already works the way I want it in that you need to send 
# something in order to receive answer back
# we already moved get barangle to method of base so now need to look at how
# send motor_msg worked but this should be simple enought and 


import serial
import time
from kite_funcs import getangle

startmarker = 60
endmarker = 62

# =====================================
#  Function Definitions
# =====================================


def init_arduino(serport='COM5', baudrate=57600):
    # NOTE the user must ensure that the serial port and baudrate are correct
    # serPort = "/dev/ttyS80"
    try:
        ser = serial.Serial(serport, baudrate)
        wait_for_arduino(ser)
    except serial.serialutil.SerialException as e:
        ser = None
    print("Serial port " + serport + " opened  Baudrate " + str(baudrate))
    return ser


def close_arduino(serial_conn):
    serial_conn.close()


def send_to_arduino(sendstr, serial_conn):
    serial_conn.write(sendstr.encode('utf-8'))  # change for Python3


def recv_from_arduino(serial_conn):
    global startmarker, endmarker
    ck = ""
    x = "z"  # any value that is not an end- or startMarker
    bytecount = -1  # to allow for the fact that the last increment will be one too many
    
    # wait for the start character
    while ord(x) != startmarker:
        x = serial_conn.read()
    
    # save data until the end marker is found
    while ord(x) != endmarker:
        if ord(x) != startmarker:
            ck = ck + x.decode("utf-8")  # change for Python3
            bytecount += 1
        x = serial_conn.read()
    return ck


def wait_for_arduino(serial_conn):
    # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
    # it also ensures that any bytes left over from a previous message are discarded
    global startmarker, endmarker
    msg = ""
    while msg.find("Arduino is ready") == -1:
        while serial_conn.inWaiting() == 0:
            pass
        msg = recv_from_arduino(serial_conn)
        print(msg)  # python3 requires parenthesis
        print('from wait for arduino')
    return


def get_sensor(ard_data):
    msg = ard_data.split(' ')
    sensor = int(msg[3])
    return sensor


def send_motor_get_barangle(base,  serial_conn):
    send_to_arduino(f'<{base.action}>', serial_conn)
    while serial_conn.inWaiting() == 0:
        pass
    datarecvd = recv_from_arduino(serial_conn)
    #print("Reply Received  " + datarecvd)
    resistance = get_sensor(datarecvd)
    barangle = getangle(resistance, base.maxleft, base.maxright,
                        base.resistleft, base.resistright, base.resistcentre)
    return resistance, barangle

# below to be deleted - not going to be used
def get_barangle(kite, base, control, config, serport):
    if config.setup == 'KiteBarActual':
        return kite.kiteangle / base.kitebarratio
    else:  # get resistance from arduino
        resistance = int(recv_from_arduino(serport))
        barangle = getangle(resistance, base.maxleft, base.maxright,
                            base.resistleft, base.resistright, base.resistcentre)
        return barangle

def runtest(td, serial_conn, sleep=1):
    print('running')
    numloops = len(td)
    waiting_for_reply = False

    n = 0
    while n < numloops:
        teststr = td[n]
        if waiting_for_reply is False:
            send_to_arduino(teststr, serial_conn)
            print("Sent from PC -- LOOP NUM " + str(n) + " TEST STR " + teststr)
            waiting_for_reply = True

        if waiting_for_reply is True:
            while serial_conn.inWaiting() == 0:
                pass
            datarecvd = recv_from_arduino(serial_conn)
            print("Reply Received  " + datarecvd)
            n += 1
            waiting_for_reply = False
            print("===========")
        time.sleep(sleep)


# Think this does a single cycle - but now rather concerned that this will need to cycle with arduino
# hopefully if we send and then receive once the sending can trigger and only aim for 1 message per cycle in sync
# with option to process more cycles if required
def call_arduino(serial_conn, motormsg: int, textback: str):
    pass


# ======================================
# THE DEMO PROGRAM STARTS HERE
# ======================================
if __name__ == "__main__":
    print()
    print()

    # NOTE the user must ensure that the serial port and baudrate are correct
    # serPort = "/dev/ttyS80"
    sp = init_arduino()
    print('back')
    testdata = ["<M, 100>", "<M, 200>", "<M, 300>", "<M, 400>", "<M, 500>", "<M, 600>"]
    runtest(testdata, sp, 0)
    bar = send_motor_get_barangle(120, sp)
    print(bar)
    sp.close()
