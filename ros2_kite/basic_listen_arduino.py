#!/usr/bin/env python
# this gets the barangle from the arduino board
# copied from basic_listen_barangle but will look to do the same via pyserial as ROS2 doesn't do serial
# aim is to have this running and continuously displaying the resistance value from arduino when run as main
# program - should be simple examples to do this
# So think we will use this as template for multi-processing in similar manner to how basic_motion_detection will
# https://docs.python.org/3/library/multiprocessing.html but most likely now just want a function there to
# read the angle and send the motor message - both will be integer
# https://www.instructables.com/Python-Serial-Port-Communication-Between-PC-and-Ar/
# https://realpython.com/arduino-python/
# https://hiteshmishra708.medium.com/multiprocessing-in-python-c6735fa70f3f
# https://www.instructables.com/Python-Serial-Port-Communication-Between-PC-and-Ar/

# below seems to be 3 key examples - think we start with first one - non threaded first
# https://forum.arduino.cc/t/demo-of-pc-arduino-comms-using-python/219184/5#msg1810764
# https://github.com/headrotor/Python-Arduino-example/blob/master/HW_Thread_py36.py threading approach
# https://github.com/pyserial/pyserial/issues/216

# TODO need to work through above links and then figure out an initial approach - suggest bottom up
# Not really convinced this is required going forward - will move to ComArduino2PY3 for now
# consider the mock stuff and then remove
from kite_funcs import getangle

barangle = 0
resistance = 200
mockresistance = 200
mockangle = 0


def callback(data):
    global resistance
    resistance = data.data
    return


def callmock(data):
    global mockresistance
    mockresistance = data.data
    return


def listen_kiteangle(message):
    if message == 'kiteangle':
        pass
        # rospy.Subscriber(message, Int16, callback, queue_size=1)
    else:
        pass
        # rospy.Subscriber(message, Int16, callmock, queue_size=1)


def get_actmockangle(kite, base, control, config):
    global mockangle, mockresistance
    mockangle = getangle(resistance, base.maxleft, base.maxright,
                         base.resistleft, base.resistright, base.resistcentre)
    return mockangle


# this should always return barangle except when barangle being set from the kite for simulation
# or on manbar when bar should be freely controlled
# This has been moved to Method of Base
def nolonger_get_barangle(kite, base, control, config):
    global barangle, resistance
    if config.setup == 'KiteBarActual':
        return kite.kiteangle / base.kitebarratio
    else:  # automated flight reading from some sort of sensor via ROS
        barangle = getangle(resistance, base.maxleft, base.maxright,
                            base.resistleft, base.resistright, base.resistcentre)
        return barangle


if __name__ == '__main__':
    listen_kiteangle('kiteangle')
