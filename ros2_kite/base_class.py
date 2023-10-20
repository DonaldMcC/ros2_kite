
# Base - this holds settings and methods of the Base/Tether setup
#   methods
#       init
#       getlogheaders
#       getlogdata
#       getcalibratetime


import time
import math
from collections import deque
from kite_funcs import getresist, conmaxright, conmaxleft, conresistleft, conresistright, conresistcentre, getangle


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


def _test():
    import doctest
    doctest.testmod()


if __name__ == '__main__':
    # Can run with -v option if you want to confirm tests were run
    _test()
