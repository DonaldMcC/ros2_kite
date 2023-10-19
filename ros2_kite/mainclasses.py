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
# Base - this holds settings and methods of the Base/Tether setup
#   methods
#       init
#       getlogheaders
#       getlogdata
#       getcalibratetime
#
# Kite

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


class Base(object):
    def __init__(self, barangle=0, parkangle=0, maxright=conmaxright, maxleft=conmaxleft, lag=1,
                 targetbarangle=0, kitebarratio=1, inferbarangle=0, resistleft=conresistleft,
                 resistright=conresistright, resistcentre=conresistcentre, safety=False):
        self.barangle = barangle
        self.parkangle = parkangle
        self.maxright = maxright
        self.maxleft = maxleft
        self.lag = lag
        self.barangles = deque(maxlen=16)
        self.targetbarangle = targetbarangle
        self.inferbarangle = inferbarangle
        self.kitebarratio = kitebarratio  # this will be the rate of change of barangle to kite angle
        self.mockangle = 0
        self.reset = False
        self.action = None
        self.resistance = 0
        self.dist_act = 35.0  # Radius from fulcrum to attachment point of actuator - we will have two actuators
        self.speed_act = 30.0
        self.actuator_length = 60
        self.calibrate = False
        self.calibrate_phase = 0
        self.start_time = 0
        self.calibrate_list = []
        self.resistright = resistright
        self.resistleft = resistleft
        self.resistcentre = resistcentre
        self.manual_calib_phases = ['Bar to straight (0 degrees) and press set or - key on wii',
                                    'Bar to 20 degrees left and press set or - key on wii',
                                    'Bar to 20 degrees right and press set or - key on wii']
        self.manual_calib_phase = 0
        self.plan_calibration()
        self.safety = safety

    @staticmethod
    def getlogheaders():
        return ('B.barangle', 'B.parkangle', 'B.maxright', 'B.maxleft', 'B.mockangle', 'B.targetbarangle',
                'B.inferbarangle', 'B.action', 'B.resistance', 'B.dist_act', 'B.speed_act', 'B.calibrate',
                'B.manual_calib_phase')

    def getlogdata(self):
        return (self.barangle, self.parkangle, self.maxright, self.maxleft, self.mockangle, self.targetbarangle,
                self.inferbarangle, self.action, self.resistance, self.dist_act, self.speed_act, self.calibrate,
                self.manual_calib_phase)

    def get_calibrate_time(self):
        # idea here is to have an expectation of how the setup should work based on components
        # believed to be there - and also identify if resistor is working as expected
        circ_act = 2 * math.pi * self.dist_act * 2  # because going to move each arm separately
        rev_time = circ_act / self.speed_act  # time for one revolution
        print(f"self.maxright=")
        return 1000 * (rev_time * self.maxright / 360.0)  # expected time to get to max angle in millisecs

    def calibration_check(self):
        # This should basically wiggle the bar and put it into position to start if results
        # not good process would be to start manual calibration from the ManBar mode
        curr_millis = round(time.monotonic() * 1000)
        elapsed_millis = curr_millis - self.start_time
        if elapsed_millis > self.calibrate_list[self.calibrate_phase][1]:
            self.calibrate_list[self.calibrate_phase][2] = elapsed_millis
            self.calibrate_list[self.calibrate_phase][4] = self.resistance
            self.start_time = curr_millis
            self.calibrate_phase += 1
            if self.calibrate_phase == 4:  # valid values are 0 to 3 this is end of loop
                self.calibrate = False
                self.calibrate_phase = 0
                self.action = 0
            self.action = self.calibrate_list[self.calibrate_phase][5]
        else:  # increase cycle counter
            self.calibrate_list[self.calibrate_phase][6] += 1
            self.calibrate_list[self.calibrate_phase][7].append(self.resistance)
        return

    def plan_calibration(self):
        #  this should initalise a list of phases that the calibration will
        #  take I think name, motormsg, target time, should  work
        target_time = int(self.get_calibrate_time())  # this is assumed to be constant for all phases
        self.action = 6

        for x, y in enumerate(range(4)):
            if (x % 2) == 0:
                action = 'fullright'
                motor_action = 6  # Left motor activated
                target_resist = getresist(self.maxright)
            else:
                action = 'centre'  # so right motor will drive
                motor_action = 7
                target_resist = getresist(0)  # should return to square
            self.calibrate_list.append([action, target_time, 0, target_resist, 0, motor_action, 0, []])
        return

    def set_resistance(self, control):
        # this should set the resistance for particular angle on the base unit and
        # is triggered when the set button is pressed
        # TODO refactor resistance into a list or dict
        if self.manual_calib_phase == 0:
            self.resistcentre = self.resistance
        elif self.resistance != self.resistcentre:  # avoid division by zero errs
            if self.manual_calib_phase == 1:
                self.resistleft = self.resistance
            else:
                self.resistright = self.resistance

        if self.manual_calib_phase < 2:
            self.manual_calib_phase += 1
        else:
            self.manual_calib_phase = 0
        print('Calibration Left Centre Right' + str(self.resistleft)
              + ' ' + str(self.resistcentre) + ' ' + str(self.resistright))
        # if self.manual_calib_phase < 2 else 0
        control.newbuttons = control.get_change_phase_buttons(self)
        return

    # this should always return barangle except when barangle being set from the kite for simulation
    # or on manbar when bar should be freely controlled
    def get_barangle(self, kite, config, resistance):
        if config.setup == 'KiteBarActual':
            self.barangle = kite.kiteangle / self.kitebarratio
        else:  # automated flight reading from some sort of sensor via ROS
            self.barangle = getangle(resistance, self.maxleft, self.maxright,
                                     self.resistleft, self.resistright, self.resistcentre)
        return

#    def update_barangle(self):
#        base.barangle = send_motor_get_barangle(base.action, serial_conn)


class Kite(object):

    def __init__(self, x=0, y=0, mode='Park', phase='Park', targetheading=0, targetangle=0, kiteangle=0):
        self.x = x
        self.y = y
        self.mode = mode
        self.phase = phase
        self.pts = deque(maxlen=16)
        self.kiteangles = deque(maxlen=16)
        self.timestamps = deque(maxlen=16)
        (self.dX, self.dY) = (0, 0)
        self.direction = ""
        self.kiteangle = kiteangle
        self.contourarea = 0
        self.zone = "Centre"
        self.targettype = 'Angle'
        self.targetx = 0
        self.targety = 0
        self.changezone = True
        self.changephase = False
        self.routechange = False
        self.found = False
        self.targetheading = targetheading
        self.targetangle = targetangle
        self.thickness = 1
        self.leftballx = 0
        self.leftbally = 0
        self.rightballx = 0
        self.rightbally = 0
        self.turncomplete = False
        self.turncomplete_angle = 60
        self.autofly = False
        return

    @staticmethod
    def getlogheaders():
        return ('K.x', 'K.y', 'K.mode', 'K.phase', 'K.direction', 'K.kiteangle', 'K.contourarea',
                'K.targettype', 'K.targetx', 'K.targety', 'K.changezone', 'K.changephase', 'K.routechange',
                'K.changephase', 'K.routechange', 'K.found', 'K.targetheading', 'K.targetangle')

    def getlogdata(self):
        return (self.x, self.y, self.mode, self.phase, self.direction, self.kiteangle, self.contourarea,
                self.targettype, self.targetx, self.targety, self.changezone, self.changephase, self.routechange,
                self.changephase, self.routechange, self.found, self.targetheading, self.targetangle)

    def get_zone(self, leftx, rightx):
        """
        >>> k=Kite(400)
        >>> k.get_zone(100,600)
        'Left'

        >>> l=Kite(400)
        >>> l.get_zone(300,600)
        'Centre'
        :param leftx:
        :param rightx:
        :return:
        """

        self.zone = 'Left' if self.x < leftx else 'Right' if self.x > rightx else 'Centre'
        return self.zone

    def get_phase(self):
        if self.mode == 'Park':
            # For park this is now OK we want to get kiteangle to zero
            self.phase = 'Hold'
        elif self.mode == 'Wiggle':
            self.phase = 'Wiggle'
        else:  # fig8 - assumed
            if self.zone == 'Centre':
                self.phase = 'Xwind'
            elif self.zone == 'Left':
                if self.turncomplete or self.kiteangle > self.turncomplete_angle:
                    self.phase = 'Xwind'
                    self.turncomplete = True
                    self.routechange = True
                else:
                    self.phase = 'TurnRight'
            else:  # Right zone
                if self.turncomplete or self.kiteangle < (0 - self.turncomplete_angle):
                    self.phase = 'Xwind'
                    self.turncomplete = True
                    self.routechange = True
                else:
                    self.phase = 'Turnleft'
        return

    def update_zone(self, control):
        currentzone = self.zone
        self.get_zone(control.routepoints[0][0], control.routepoints[3][0])
        self.changezone = True if self.zone != currentzone else False
        if self.changezone:  # set to false at start of next turn
            self.turncomplete = False
        return

    def update_phase(self):
        currentphase = self.phase
        self.get_phase()
        self.changephase = True if self.phase != currentphase else False
        return

    def get_wiggle_angle(self):
        x = -10 if self.kiteangle > 0 else 10
        return x

    def update_target(self, leftx, lefty, centrex, centrey, rightx, righty):
        # this gets called when mode, zone, phase or route changes
        if self.mode == 'Park':
            # For park this is now OK we want to get kiteangle to zero
            self.targettype = 'Angle'
            self.targetangle = 0
            self.targetx = centrex
            self.targety = centrey
        elif self.mode == 'Wiggle':
            self.targettype = 'Angle'
            self.targetangle = self.get_wiggle_angle()
            self.targetx = centrex
            self.targety = centrey
        else:  # fig8 - by definition
            if self.zone == 'Centre' or self.phase == 'Xwind':
                # Either we have just left the right or left turnzone so if nearest
                # left we go right and if nearest right we go left
                # or we have changed from park or wiggle to xwind which will be presumed to happen
                # with kite upwards and seems reasonable to just go for the longer xwind distance
                self.targettype = 'Point'
                if abs(self.x - leftx) > abs(self.x - rightx):
                    self.targetx = leftx
                    self.targety = lefty
                else:
                    self.targetx = rightx
                    self.targety = righty
                # self.targetangle = get_heading_points((self.x, self.y), (self.targetx, self.targety))
            elif self.changezone:  # think we should still set this roughly in the turn phase
                self.targettype = self.phase
                if self.phase == 'TurnR':
                    self.targetangle = 90
                else:
                    self.targetangle = -90
                # TODO - may compute the target location
            else:
                print('End of update_target reached without cover expected cases most likely')
                # TODO ensure change of flight mode is barred unless in the centre zone -
                # seems sensible and should
                # mean changemode and changephase generally only triggered in centre zone
        return

    def move_kite(self, control, speed=10):
        # This moves a manual kite in autofly mode towards the target while in the centre zone - however when
        # in turn the target only moves when we get back out of the turn zone so instead we probaby want to pick up
        # the apex of fig 8 and then on up to the top which we just fly through and then the target will change again
        # pass lets start with figuring out the actual targetx and targety
        if self.zone == 'Centre':
            movex, movey = self.targetx, self.targety
            # want to move beyond the target so extend x
        elif self.zone == 'Left':  # this just takes us to top of zone
            movex, movey = control.routepoints[3]
        else:  # Right
            movex, movey = control.routepoints[0]

        # Ensure we go past the point which toggles the change of zone
        adjustx = 20 if movex >= control.centrex else -20
        movex += adjustx
        self.x, self.y = move_item(self.x, self.y, movex, movey, speed)
        return


def _test():
    import doctest
    doctest.testmod(extraglobs={'k': Kite()})


if __name__ == '__main__':
    # Can run with -v option if you want to confirm tests were run
    _test()
