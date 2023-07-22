import serial
import time
serialport = serial.Serial(port='', baudrate=115200, timeout=0, rtscts=True)

startmarker = '<'
endmarker = '>'
datastarted = False
databuf = ""
messagecomplete = False


# the functions
def setup_serial(baudrate, serialportname):
    global serialport
    serialport = serial.Serial(port=serialportname, baudrate=baudrate, timeout=0, rtscts=True)
    print("Serial port " + serialportname + " opened  Baudrate " + str(baudrate))
    wait_arduino()


def send_arduino(string_send):
    # this adds the start- and end-markers before sending
    global startmarker, endmarker, serialport
    stringwithmarkers = startmarker + string_send + endmarker
    serialport.write(stringwithmarkers.encode('utf-8'))  # encode needed for Python3
    return


def recv_arduino():
    global startmarker, endmarker, serialport, datastarted, databuf, messagecomplete
    if serialport.inWaiting() > 0 and messagecomplete is False:
        x = serialport.read().decode("utf-8")  # decode needed for Python3
        if datastarted is True:
            if x != endmarker:
                databuf = databuf + x
            else:
                datastarted = False
                messagecomplete = True
        elif x == startmarker:
            databuf = ''
            datastarted = True

    if messagecomplete is True:
        messagecomplete = False
        return databuf
    else:
        return "XXX"


def wait_arduino():
    # wait until the Arduino sends 'Arduino is ready' - allows time for Arduino reset
    # it also ensures that any bytes left over from a previous message are discarded
    print("Waiting for Arduino to reset")
    msg = ""
    while msg.find("Arduino is ready") == -1:
        msg = recv_arduino()
        if not (msg == 'XXX'):
            print(msg)


if __name__ == "__main__":
    # the program
    setup_serial(115200, "COM5")
    count = 0
    prevTime = time.time()
    while True:
        # check for a reply
        arduinoReply = recv_arduino()
        if not (arduinoReply == 'XXX'):
            print("Time %s  Reply %s" % (time.time(), arduinoReply))

            # send a message at intervals
        if time.time() - prevTime > 1.0:
            send_arduino("this is a test " + str(count))
            prevTime = time.time()
            count += 1
