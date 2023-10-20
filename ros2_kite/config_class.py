#!/usr/bin/env python
#
"""
This file should do the following things
    1   Calculate a default route for the kite based on settings or parameters applied - done
    2   Identify the flight zone which will be left, right or centre in fig8
        and park_left, park_right - done
    3   Identify the flight mode - currently proposing park wiggle and fig8up we always start with park
    4   Has phase change or zone changed or are we starting
    3   Set route - otherwise we know the preferred route
    4   Staring point will always be park and the target for that is kite angle of zero and to be at top of centre line
    5   Probably then go left a bit and right a bit - lets call that wiggle mode - but we align first
    6   Then move into fig 8 with upturns - let's always start left and should be aimed high - probably just need
        to display a centre line and always draw fig 8 and resize manually for now - full automation of that can
        be later - this will be set in motion manually
    7   Once there we flick into upturn and measure turn radius for a few cycles - turn stops when kite is round 180deg
    8   Then repeat to other side -
    9   At some point we would switch to doing down turns but that can probably be well after upturns work reliably so
    10  Upturns only for now
"""

# There are 4 main classes for the kite related objects and they have following methods
# Config - this holds settings for the operation
#   methods:
#       init
#       getlogheaders - outputs headers for CSV Log File
#       getlogdata - outputs values for log file



import time
import math
from collections import deque
from kite_funcs import getresist, conmaxright, conmaxleft, conresistleft, conresistright, conresistcentre, getangle
from move_func import move_item


class Config(object):
    def __init__(self, source=2, kite='Standard', masklimit=10000,
                 logging=0, numcams=1, check_motor_sim=False, setup='Standard', input='Keyboard'):
        self.source = source
        self.kite = kite
        self.masklimit = masklimit
        self.logging = logging
        self.numcams = numcams
        self.check_motor_sim = check_motor_sim
        self.setup = setup
        self.input = input
        self.writer = None

    @staticmethod
    def getlogheaders():
        return 'source', 'kite', 'masklimit', 'numcams', 'check_motor_sim', 'setup'

    def getlogdata(self):
        return self.source, self.kite, self.masklimit, self.numcams, self.check_motor_sim, self.setup



def _test():
    import doctest
    doctest.testmod(extraglobs={'k': Kite()})


if __name__ == '__main__':
    # Can run with -v option if you want to confirm tests were run
    _test()
