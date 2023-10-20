#!/usr/bin/env python
#
# Kite class provides the following methods
#   init - sets up the kite and main properties
#   getlogheaders - outputs headers to log to csv file
#   getlogdata - outputs the actual values to log data to csv file
#   get_zone
#   get_phase
#   update_zone
#   update_phase
#   get_wiggle_angle
#   update_target
#   move_kite


from collections import deque
from move_func import move_item


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
