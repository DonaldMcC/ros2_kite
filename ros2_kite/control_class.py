#!/usr/bin/env python
#
# Controls class handles the routing and inputs from keyboard or joystick to
# the program.  Joystock was linked to wiimote when using ROS - not planning
# on using that going forward so will be keyboard and mouse input going forward
# along with keyboard, mouse and joystick there is suport for 4 input 
import time
from kite_funcs import calc_route


class Controls(object):

    def __init__(self, config='Standard', step=8, motortest=False):

        #  mode='fig8'
        self.centrex = 400
        self.centrey = 300
        self.halfwidth = 200
        self.radius = 100
        self.routepoints = calc_route(self.centrex, self.centrey, self.halfwidth, self.radius)
        self.config = config  # possible config ('Standard', 'Manual', 'Manbar')
        self.inputmode = 0 if self.config == 'Standard' else 2 if self.config == 'Manual' else 3
        self.step = step
        self.modestring = self.getmodestring(True)
        self.route = False
        self.maxy = 20  # this should be for top of centre line and sets they y target point for park mode
        self.slow = 0.0
        self.newbuttons = []
        self.motortest = motortest

    @staticmethod
    def getlogheaders():
        return 'C.config', 'C.inputmode', 'C.motortest'

    def getlogdata(self):
        return self.config, self.inputmode, self.motortest

    def getmodestring(self, inputmode):
        # So now always 11 buttons and first 5 and last 2 are std and iteration through should be std 
        # so we would have a defined transition of names based on which change took place
        if inputmode == 0:  # Standard
            return 'STD: Left Right Up Down Pause Wider Narrow Expand Contract Mode Quit'
        elif inputmode == 1:
            return 'SETFLIGHTMODE: Left Right Up Down Pause Park Wiggle Fig8 Reset Mode Quit'
        elif inputmode == 2:
            return 'MANFLIGHT: Left Right Up Down Pause Anti Clock Slow Fast Mode Quit'
        else:  # inputmode = 3
            return 'MANBAR: Left Right Up Down Pause Anti Clock Slow Fast Mode Quit'

    @staticmethod
    def get_change_mode_buttons(inputmode):
        if inputmode == 0:  # STD
            newbuttons = [('Mode: STD:', 'Mode: STD:'), ('Pause', 'Pause'), ('Wider', 'Wider'), ('Narrow', 'Narrow'),
                          ('Expand', 'Expand'), ('Contract', 'Contract')]
        elif inputmode == 1:  # SETFLIGHTMODE
            newbuttons = [('Mode: STD:', 'Mode: SETFLIGHTMODE:'), ('Wider', 'Park'), ('Narrow', 'Wiggle'),
                          ('Expand', 'Fig8'), ('Contract', 'Autofly')]
        elif inputmode == 2:
            newbuttons = [('Mode: STD:', 'Mode: MANFLIGHT'), ('Wider', 'Anti'), ('Narrow', 'Clock'),
                          ('Expand', 'Slow'), ('Contract', 'Fast')]
        else:  # MANBAR
            newbuttons = [('Mode: STD:', 'Mode: MANBAR:'), ('Pause', 'Calib'), ('Expand', 'Set')]
        return newbuttons

    @staticmethod
    def get_change_phase_buttons(base):
        newbuttons = [('Mode: STD:', 'Mode:' + base.manual_calib_phases[base.manual_calib_phase])]
        return newbuttons

    def joyhandler(self, joybuttons, joyaxes, kite, base, control, event=None):
        # Using https://github.com/arnaud-ramey/rosxwiimote as a ros package to capture
        # the joystick message this was because std one tried to do bluetooth
        # connection to wiimote via python and it didn't work perhaps as only
        # seems to expect early versions of wiimote

        # The axes messages is as follows:
        # 0. left - right rocker(3 possible values: -1 = left 0 = released 1 = right)
        # 1. up - down rocker(3 possible values: -1 = left 0 = released 1 = right)
        # 2. nunchuk left - right joystick(floating value in the range - 1 = left..1 = right)
        # 3. nunchuk down - up joystick(floating value in the range - 1 = down.. 1 = up)

        # 0. XWII_KEY_A - this should change the mode
        # 1. XWII_KEY_B - this should toggle the rockers between move and stretch squashc
        # 2. XWII_KEY_PLUS - probably the faster button and poss some other things for playback
        # 3. XWII_KEY_MINUS - probably the slower button and poss some other things for playback in slow motion
        # 4. XWII_KEY_HOME this should be the quit key
        # 5. XWII_KEY_ONE  this will do a pause or calibrate in manbar mode
        # 6. XWII_KEY_TWO  and this will do a flight mode change
        # 7. XWII_KEY_C - so
        # 8. XWII_KEY_Z   so bigger numchuck key should change manflight to angle kite

        # in terms of what we do with this the basic idea is that the nunchuk flies the kite
        # and the rockers support the route moving about
        reset_stitcher = False

        # events for all input modes
        if (joybuttons and joybuttons[0] == 1) or event == 'Mode':  # modechange on A key
            self.inputmode += 1
            kite.barbasedangle = True if self.inputmode == 3 else False
            if self.inputmode == 4:  # simple toggle around 3 modes
                self.inputmode = 0
            self.modestring = self.getmodestring(self.inputmode)
            self.newbuttons = self.get_change_mode_buttons(self.inputmode)
            base.calibrate = False  # calibration always ends on Mode Change
        elif (joybuttons and joybuttons[5] == 1) or event == 'Pause':  # pause on 1 key
            if control.inputmode == 3:  # Manbar Calibrate
                base.calibrate = 'Manual'  # then slow button becomes set to set and that actions and cycles
                base.safety = False  # if starts true
                self.newbuttons = self.get_change_phase_buttons(base)
            elif not control.motortest:
                time.sleep(10)
            else:
                base.action = 500  # Stop
        elif joybuttons and joybuttons[3] == 1:  # slow
            self.slow += 0.1
        elif joybuttons and joybuttons[2] == 1:  # fast
            self.slow = 0.0

        # common handling when not in one of the man modes
        if self.inputmode == 0 or self.inputmode == 1:
            if (joybuttons and joyaxes[0] == -1) or event == 'Left':  # left:  # left
                if not control.motortest:
                    self.centrex -= self.step
                    kite.routechange = True
                else:
                    base.action = 300
            elif (joybuttons and joyaxes[0] == 1) or event == 'Right':  # right
                if not control.motortest:
                    self.centrex += self.step
                    kite.routechange = True
                else:
                    base.action = 400
            elif (joybuttons and joyaxes[1] == 1) or event == 'Up':  # up
                if not control.motortest:
                    self.centrey -= self.step
                    kite.routechange = True
                else:
                    base.action = 100
            elif (joybuttons and joyaxes[1] == -1) or event == 'Down':  # down
                if not control.motortest:
                    self.centrey += self.step
                    kite.routechange = True
                else:
                    base.action = 200
        elif self.inputmode == 2 or self.inputmode == 3:  # common events for Man modes
            if joybuttons:
                if joyaxes[0] != 0:  # -1 = left +1 = right
                    self.centrex += self.step * int(joyaxes[0])
                    kite.routechange = True
                elif joyaxes[1] != 0:  # 1 = up -1 = down so needs inverted
                    self.centrey -= self.step * int(joyaxes[1])
                    kite.routechange = True
            if event == 'Left':  # left
                kite.x -= self.step
            elif event == 'Right':  # right
                kite.x += self.step
            elif event == 'Up':  # up
                kite.y -= self.step
            elif event == 'Down':  # down
                kite.y += self.step
            elif event == 'Expand' or (joybuttons and joybuttons[3] == 1):  # slow
                if base.calibrate != 'Manual':
                    self.slow += 0.1
                else:
                    base.set_resistance(control)
                    time.sleep(0.5)
            elif event == 'Contract':  # fast
                self.slow = 0.0

        if self.inputmode == 0:  # Standard
            if event == 'Wider':  # wider
                self.halfwidth += self.step
            elif event == 'Narrow':  # narrower
                self.halfwidth -= self.step
            elif event == 'Expand':  # expand
                self.radius += self.step
            elif event == 'Contract':  # contract
                self.radius -= self.step
        elif self.inputmode == 1:  # SetFlight
            if joybuttons and joybuttons[6] == 1:  # move mode forward
                if kite.mode == 'Park':
                    kite.mode = 'Wiggle'
                elif kite.mode == 'Wiggle':
                    kite.mode = 'Fig8'
                else:
                    kite.mode = 'Park'
            if event == 'Wider':  # park
                kite.mode = 'Park'
            elif event == 'Narrow' and kite.zone == 'Centre':  # must be in central zone to change mode
                kite.mode = 'Wiggle'
            elif event == 'Expand' and kite.zone == 'Centre':  # must be in central zone to change mode
                kite.mode = 'Fig8'
            elif event == 'Contract':  # Autofly
                # base.reset = True # don't think this ever did anything
                kite.autofly = True if kite.autofly is False else False
        elif self.inputmode == 2:  # ManFlight - maybe switch to arrows - let's do this all
            if joybuttons:
                if joybuttons[7] == 0 and joybuttons[8] == 0:
                    kite.x += (self.step * joyaxes[2])
                    kite.y -= (self.step * joyaxes[3])
                else:  # c or z button pressed angle the kite and automatically the bar
                    kite.kiteangle += (self.step / 2 * joyaxes[2])
            if event == 'Wider':  # anti clockwise
                kite.kiteangle -= self.step
            elif event == 'Narrow':  # clockwise
                kite.kiteangle += self.step
        elif self.inputmode == 3:  # ManBar - maybe switch to arrows - let's do this all
            if joybuttons:
                if joybuttons[7] == 0 and joybuttons[8] == 0:
                    if joyaxes[2] < -0.1:
                        base.action = int(300 - (joyaxes[2] * 99))
                    elif joyaxes[2] > 0.1:
                        base.action = int(400 + (joyaxes[2] * 99))
                    elif joyaxes[3] > 0.1:
                        base.action = int(100 + (joyaxes[3] * 99))
                    elif joyaxes[3] < -0.1:
                        base.action = int(200 - (joyaxes[3] * 99))
                    else:
                        base.action = 0
                    print(base.action)
                else:  # c or z button pressed
                    kite.x += (self.step * joyaxes[2])
                    kite.y -= (self.step * joyaxes[3])
            else:
                if event == 'Wider':  # anti-clockwise
                    base.action = 300
                elif event == 'Narrow':  # clockwise
                    base.action = 400
                else:
                    base.action = 0

        return joybuttons and joybuttons[4] == 1, reset_stitcher  # quit

    def keyhandler(self, key, kite, base, control, event):
        # Relifted from old version temporarily
        # this will now support a change of flight mode and operating mode so different keys will
        # do different things depending on inputmode,
        reset_stitcher = False
        # events for all input modes

        if event == 'Mode' or key == ord("m"):  # modechange on A key
            self.inputmode += 1
            kite.barbasedangle = True if self.inputmode == 3 else False
            if self.inputmode == 4:  # simple toggle around 3 modes
                self.inputmode = 0
            self.modestring = self.getmodestring(self.inputmode)
            self.newbuttons = self.get_change_mode_buttons(self.inputmode)
            base.calibrate = False  # calibration always ends on Mode Change
        elif event == 'Pause':  # pause on 1 key
            if control.inputmode == 3:  # Manbar Calibrate
                base.calibrate = 'Manual'  # then slow button becomes set to set and that actions and cycles
                base.safety = False  # if starts true
                self.newbuttons = self.get_change_phase_buttons(base)
            elif not control.motortest:
                time.sleep(10)
            else:
                base.action = 500  # Stop

        # common handling when not in one of the man modes
        if self.inputmode == 0 or self.inputmode == 1:
            if event == 'Left' or key == ord("l"):  # left:  # left
                if not control.motortest:
                    self.centrex -= self.step
                    kite.routechange = True
                else:
                    base.action = 300
            elif event == 'Right' or key == ord("r"):  # right
                if not control.motortest:
                    self.centrex += self.step
                    kite.routechange = True
                else:
                    base.action = 400
            elif event == 'Up' or key == ord("u"):  # up
                if not control.motortest:
                    self.centrey -= self.step
                    kite.routechange = True
                else:
                    base.action = 100
            elif event == 'Down' or key == ord("d"):  # down
                if not control.motortest:
                    self.centrey += self.step
                    kite.routechange = True
                else:
                    base.action = 200
        elif self.inputmode == 2 or self.inputmode == 3:  # common events for Man modes
            if event == 'Left' or key == ord("l"):  # left
                kite.x -= self.step
            elif event == 'Right' or key == ord("r"):  # right
                kite.x += self.step
            elif event == 'Up' or key == ord("u"):  # up
                kite.y -= self.step
            elif event == 'Down' or key == ord("d"):  # down
                kite.y += self.step
            elif event == 'Expand' or key == ord("e"):  # slow
                if base.calibrate != 'Manual':
                    self.slow += 0.1
                else:
                    base.set_resistance(control)
                    time.sleep(0.5)
            elif event == 'Contract' or key == ord("c"):  # fast
                self.slow = 0.0

        if self.inputmode == 0:  # Standard
            if event == 'Wider' or key == ord("w"):  # wider
                self.halfwidth += self.step
            elif event == 'Narrow' or key == ord("n"):  # narrower
                self.halfwidth -= self.step
            elif event == 'Expand' or key == ord("e"):  # expand
                self.radius += self.step
            elif event == 'Contract' or key == ord("c"):  # contract
                self.radius -= self.step
        elif self.inputmode == 1:  # SetFlight
            if event == 'Wider' or key == ord("p"):  # park
                kite.mode = 'Park'
            elif (event == 'Narrow' or key == ord("w")) and kite.zone == 'Centre':
                # must be in central zone to change mode
                kite.mode = 'Wiggle'
            elif (event == 'Expand' or key == ord("e")) and kite.zone == 'Centre':
                # must be in central zone to change mode
                kite.mode = 'Fig8'
            elif event == 'Contract' or key == ord("c"):  # Autofly
                # base.reset = True # don't think this ever did anything
                kite.autofly = True if kite.autofly is False else False
        elif self.inputmode == 2:  # ManFlight - maybe switch to arrows - let's do this all
            if event == 'Wider' or key == ord("w"):  # anti clockwise
                kite.kiteangle -= self.step
            elif event == 'Narrow' or key == ord("n"):  # clockwise
                kite.kiteangle += self.step
        elif self.inputmode == 3:  # ManBar - maybe switch to arrows - let's do this all
            if event == 'Wider' or key == ord("w"):  # anti-clockwise
                base.action = 300
            elif event == 'Narrow' or key == ord("n"):  # clockwise
                base.action = 400
            else:
                base.action = 0
        return key == ord("q"), reset_stitcher  # quit

    def mousehandler(self, kite, base, control, event=None):
        # This will hopefully remove the joystick code to allow using with mouse and less code
        # whole routine below complicated and want to look at mousehandling later so strip back
        # first
        reset_stitcher = False

        # events for all input modes
        if event == 'Mode':  # modechange on A key
            self.inputmode += 1
            kite.barbasedangle = True if self.inputmode == 3 else False
            if self.inputmode == 4:  # simple toggle around 3 modes
                self.inputmode = 0
            self.modestring = self.getmodestring(self.inputmode)
            self.newbuttons = self.get_change_mode_buttons(self.inputmode)
            base.calibrate = False  # calibration always ends on Mode Change
        elif event == 'Pause':  # pause on 1 key
            if control.inputmode == 3:  # Manbar Calibrate
                base.calibrate = 'Manual'  # then slow button becomes set to set and that actions and cycles
                base.safety = False  # if starts true
                self.newbuttons = self.get_change_phase_buttons(base)
            elif not control.motortest:
                time.sleep(10)
            else:
                base.action = 500  # Stop

        # common handling when not in one of the man modes
        if self.inputmode == 0 or self.inputmode == 1:
            if event == 'Left':  # left:  # left
                if not control.motortest:
                    self.centrex -= self.step
                    kite.routechange = True
                else:
                    base.action = 300
            elif event == 'Right':  # right
                if not control.motortest:
                    self.centrex += self.step
                    kite.routechange = True
                else:
                    base.action = 400
            elif event == 'Up':  # up
                if not control.motortest:
                    self.centrey -= self.step
                    kite.routechange = True
                else:
                    base.action = 100
            elif event == 'Down':  # down
                if not control.motortest:
                    self.centrey += self.step
                    kite.routechange = True
                else:
                    base.action = 200
        elif self.inputmode == 2 or self.inputmode == 3:  # common events for Man modes
            if event == 'Left':  # left
                kite.x -= self.step
            elif event == 'Right':  # right
                kite.x += self.step
            elif event == 'Up':  # up
                kite.y -= self.step
            elif event == 'Down':  # down
                kite.y += self.step
            elif event == 'Expand':  # slow
                if base.calibrate != 'Manual':
                    self.slow += 0.1
                else:
                    base.set_resistance(control)
                    time.sleep(0.5)
            elif event == 'Contract':  # fast
                self.slow = 0.0

        if self.inputmode == 0:  # Standard
            if event == 'Wider':  # wider
                self.halfwidth += self.step
            elif event == 'Narrow':  # narrower
                self.halfwidth -= self.step
            elif event == 'Expand':  # expand
                self.radius += self.step
            elif event == 'Contract':  # contract
                self.radius -= self.step
        elif self.inputmode == 1:  # SetFlight
            if event == 'Wider':  # park
                kite.mode = 'Park'
            elif event == 'Narrow' and kite.zone == 'Centre':  # must be in central zone to change mode
                kite.mode = 'Wiggle'
            elif event == 'Expand' and kite.zone == 'Centre':  # must be in central zone to change mode
                kite.mode = 'Fig8'
            elif event == 'Contract':  # Autofly
                # base.reset = True # don't think this ever did anything
                kite.autofly = True if kite.autofly is False else False
        elif self.inputmode == 2:  # ManFlight - maybe switch to arrows - let's do this all
            if event == 'Wider':  # anti clockwise
                kite.kiteangle -= self.step
            elif event == 'Narrow':  # clockwise
                kite.kiteangle += self.step
        elif self.inputmode == 3:  # ManBar - maybe switch to arrows - let's do this all
            if event == 'Wider':  # anti-clockwise
                base.action = 300
            elif event == 'Narrow':  # clockwise
                base.action = 400
            else:
                base.action = 0
        return False, reset_stitcher  # quit
