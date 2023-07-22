import serial
import time
serialPort = None

startMarker = '<'
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False


# the functions
def setupSerial(baudRate, serialPortName):
    global serialPort
    serialPort = serial.Serial(port=serialPortName, baudrate=baudRate, timeout=0, rtscts=True)
    print("Serial port " + serialPortName + " opened  Baudrate " + str(baudRate))
    waitForArduino()


def sendToArduino(stringToSend):
    # this adds the start- and end-markers before sending
    global startMarker, endMarker, serialPort
    stringWithMarkers = startMarker
    stringWithMarkers += stringToSend
    stringWithMarkers += endMarker
    serialPort.write(stringWithMarkers.encode('utf-8'))  # encode needed for Python3
    return


def recvLikeArduino():
    global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete
    if serialPort.inWaiting() > 0 and messageComplete is False:
        x = serialPort.read().decode("utf-8")  # decode needed for Python3
        if dataStarted is True:
            if x != endMarker:
                dataBuf = dataBuf + x
            else:
                dataStarted = False
                messageComplete = True
        elif x == startMarker:
            dataBuf = ''
            dataStarted = True

    if messageComplete is True:
        messageComplete = False
        return dataBuf
    else:
        return "XXX"


def waitForArduino():
    # wait until the Arduino sends 'Arduino is ready' - allows time for Arduino reset
    # it also ensures that any bytes left over from a previous message are discarded
    print("Waiting for Arduino to reset")
    msg = ""
    while msg.find("Arduino is ready") == -1:
        msg = recvLikeArduino()
        if not (msg == 'XXX'):
            print(msg)


# the program
setupSerial(115200, "COM5")
count = 0
prevTime = time.time()
while True:
    # check for a reply
    arduinoReply = recvLikeArduino()
    if not (arduinoReply == 'XXX'):
        print("Time %s  Reply %s" % (time.time(), arduinoReply))

        # send a message at intervals
    if time.time() - prevTime > 1.0:
        sendToArduino("this is a test " + str(count))
        prevTime = time.time()
        count += 1
