#!/usr/bin/env python
# this gets the barangle from the arduino board
# copied from basic_listen_barangle but will look to do the same via pyserial as ROS2 doesn't do serial
# aim is to have this running and continuously displaying the resistance value from arduino when run as main
# program - should be simple examples to do this
# So think we will use this as template for multi-processing in similar manner to how basic_motion_detection will
# https://docs.python.org/3/library/multiprocessing.html but most likely now just want a function there to
# read the angle and send the motor message - both will be integer

from multiprocessing import Process, Value, Array
from kite_funcs import getangle


def f(n, a):
    n.value = 3.1415927
    for i in range(len(a)):
        a[i] = -a[i]


if __name__ == '__main__':
    num = Value('d', 0.0)
    arr = Array('i', range(10))

    p = Process(target=f, args=(num, arr))
    p.start()
    p.join()

    print(num.value)
    print(arr[:])


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
def get_barangle(kite, base, control, config):
    global barangle, resistance
    if config.setup == 'KiteBarActual':
        return kite.kiteangle / base.kitebarratio
    else:  # automated flight reading from some sort of sensor via ROS
        barangle = getangle(resistance, base.maxleft, base.maxright,
                            base.resistleft, base.resistright, base.resistcentre)
        return barangle


if __name__ == '__main__':
    listen_kiteangle('kiteangle')
