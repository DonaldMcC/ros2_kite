#!/usr/bin/env python
# this will be the main module for configuring the desired route and
# for showing the position and angle of the kite
# it is the only video output from the package and consequently
# will display and also allow direct updating of the proposed route

# inputs
# the module will support main input either input single or dual webcam or from a video file - this may
# extend to rosbag files in future
# there is input from a resistor which is linked to the kitebar and can be calibrated to provide
# the angle of the bar which is now being moved via an arduino connect to two actuators
#
# outputs
# the main output will be a ROS message reporting the x and y coordinates of the kite,
# the current angle of the control bar and the motor instruction to change the angle of the bar.
# It is possible to record the input if required - the motor instruction is on motormsg.data
#
# initial configuration
# file can be started with  various arguments and can be easily changed to work with a specified
# video file if started without arguments it will generally use webcam input - however this can
# change depending on what I am working on.
#
# while runing it is possible there are 4 modes
# 1 Std - the main controls adjust the height and shape of figure of 8 routing
# 2 SetFlightMode - starting objective is stability of kite in park position - but also wiggle
#   and then fig 8 are possible
# 2 Manflight - the main controls adjust the position and angle of a manual kite
# 3 Manbar - controls adjust the angle of the bar as well as key for the rout
#
# At startup it is possible to:
# 1 detect an actual kite on video or manually controlled one for testing and simulation
# 2 set the linkage between controls - Standard, BarKiteActual, KiteBarInfer, KiteBarTarget
# Standard means no connections between KiteAngle, KiteTargetAngle and Bar Angles
# KiteAngle means the observed or actual angle of the kite sets the actual bar angle
# KiteTargetAngle - observed or actual angle of the kite sets the target bar angle
# BarAngle - the actual angle of the bar sets the angle of the kite - this setup can
# be achieved at any point by going into Manbar - but BarAngle basically starts with this setup
# show connections from and to ie BarKiteActual the Kite angle is updated from the bar Angle
#
#
# on playback it should be possible to go into slow motion

use_ros2=False

# standard library imports
import numpy as np
import PySimpleGUI as sg
import time
import cv2
import argparse
# pyimagesearch imports
from imutils.video import VideoStream
from panorama import Stitcher
import imutils
# kite_ros imports
from move_func import get_heading_points, get_angled_corners
from mainclasses import Kite, Controls, Base, Config, calc_route
from move_func import get_angle
if use_ros2:
    from talker import motor_msg, init_motor_msg, init_ros
    from basic_listen_barangle import listen_kiteangle, get_actmockangle
    from listen_joystick import listen_joystick, get_joystick
from kite_funcs import kitemask, get_action, get_angles
import PID
from kite_logging import writelogs, writelogheader, writepictheader, closelogs


def drawroute(route, centrex, centrey):
    global frame
    for k, l in enumerate(route):
        if k < len(route) - 1:
            cv2.line(frame, (l[0], l[1]), (route[k + 1][0], route[k + 1][1]),
                     (0, 255, 70), thickness=2, lineType=8, shift=0)
        else:
            cv2.line(frame, (l[0], l[1]), (route[0][0], route[0][1]),
                     (0, 255, 70), thickness=2, lineType=8, shift=0)
    cv2.line(frame, (centrex, 0), (centrex, centrey * 2),
             (255, 0, 0), thickness=2, lineType=8, shift=0)
    return


def drawcross(manx, many, crosstype='Man', colour=(255, 0, 255)):
    global frame  #
    # stuff below was to allow angle calculation of angle - which may well
    # do once we have got direction of travel unpicked
    if crosstype == 'Man':
        crosssize = 10
        thickness = 2
    else:  # Target for now
        crosssize = 9
        thickness = 3
    starthorx = manx - crosssize
    endhorx = manx + crosssize
    endhory = many
    starthory = many
    startvery = many - crosssize
    endvery = many + crosssize
    endverx = manx
    startverx = manx
    cv2.line(frame, (starthorx, starthory), (endhorx, endhory), colour, thickness=thickness, lineType=8, shift=0)
    cv2.line(frame, (startverx, startvery), (endverx, endvery), colour, thickness=thickness, lineType=8, shift=0)
    return


def drawkite(kite):
    global frame  #
    # stuff below was to allow angle calculation of angle - which may well
    # do once we have got direction of travel unpicked
    height = 20
    width = 20
    thickness = 2
    colour = (0, 255, 255)
    starthorx = kite.x - width
    endhorx = kite.x + width
    endhory = kite.y
    starthory = kite.y
    startvery = kite.y - height
    endvery = kite.y + height * 2
    endverx = kite.x
    startverx = kite.x
    starthorx, starthory = get_angled_corners(starthorx, starthory, kite.kiteangle, kite.x, kite.y, 'int')
    endhorx, endhory = get_angled_corners(endhorx, endhory, kite.kiteangle, kite.x, kite.y, 'int')

    startverx, startvery = get_angled_corners(startverx, startvery, kite.kiteangle, kite.x, kite.y, 'int')
    endverx, endvery = get_angled_corners(endverx, endvery, kite.kiteangle, kite.x, kite.y, 'int')

    cv2.line(frame, (starthorx, starthory), (endhorx, endhory), colour, thickness=thickness, lineType=8, shift=0)
    cv2.line(frame, (startverx, startvery), (endverx, endvery), colour, thickness=thickness, lineType=8, shift=0)
    cv2.line(frame, (starthorx, starthory), (endverx, endvery), colour, thickness=thickness, lineType=8, shift=0)
    cv2.line(frame, (endverx, endvery), (endhorx, endhory), colour, thickness=thickness, lineType=8, shift=0)
    cv2.line(frame, (endhorx, endhory), (startverx, startvery), colour, thickness=thickness, lineType=8, shift=0)
    cv2.line(frame, (startverx, startvery), (starthorx, starthory), colour, thickness=thickness, lineType=8, shift=0)
    return


def getdirection(kte):
    # start direction and analysis - this will be a routine based on class
    for i in np.arange(1, len(kte.pts)):
        if kte.pts[i] is None:
            continue
        # check to see if enough points have been accumulated in the buffer
        if counter >= 10 and i == 1 and kte.pts and len(kte.pts) > 10 and kte.pts[10] is not None:
            # compute the difference between the x and  y  coordinates and re-initialize the direction
            # text variables
            kte.dX = kte.pts[i][0] - kte.pts[-10][0]
            kte.dY = kte.pts[i][1] - kte.pts[-10][1]
            (dirX, dirY) = ("", "")

            # ensure there is significant movement in the x-direction
            if np.abs(kte.dX) > 20:
                dirX = "East" if np.sign(kte.dX) == 1 else "West"
            # ensure there is significant movement in the y-direction
            if np.abs(kte.dY) > 20:
                dirY = "South" if np.sign(kte.dY) == 1 else "North"
                # handle when both directions are non-empty
            if dirX != "" and dirY != "":
                kte.direction = f"{dirY}-{dirX}"
                # otherwise, only one direction is non-empty
            else:
                kte.direction = dirX if dirX != "" else dirY

            # otherwise, compute the thickness of the line and draw the connecting lines
            kte.thickness = int(np.sqrt(32 / float(i + 1)) * 2.5)
            cv2.line(frame, kte.pts[i - 1], kte.pts[i], (0, 0, 255), kte.thickness)
            continue
    return


def display_base(width):
    outx = width - 180
    centx = outx + 60
    centy = 300
    radius = 60
    cv2.putText(frame, 'Base R:' + str(base.resistance), (outx + 20, centy - 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
    cv2.circle(frame, (centx, centy), radius, (0, 255, 255), 2)
    cv2.putText(frame, 'Act: ' + f'{base.barangle:5.1f}', (outx + 15, centy + 100), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (0, 255, 0), 2)
    cv2.putText(frame, 'Tgt: ' + f'{base.targetbarangle:5.1f}', (outx + 15, centy + 130),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    display_line(base.targetbarangle, centx, centy, radius, (0, 255, 255))
    display_line(base.barangle, centx, centy, radius, (0, 255, 0))
    if config.setup == 'KiteBarInfer':
        display_line(base.inferbarangle, centx, centy, radius, (255, 0, 0))
        cv2.putText(frame, 'Inf: ' + f'{base.inferbarangle:5.1f}', (outx + 15, centy + 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2)
    if config.check_motor_sim:
        display_line(base.mockangle, centx, centy, radius, (128, 0, 0))
        cv2.putText(frame, 'Mock: ' + f'{base.mockangle:5.1f}', (outx + 15, centy + 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (128, 0, 0), 2)
    return


def display_stats():
    cv2.putText(frame, kite.direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
    cv2.putText(frame, f"dx: {kite.dX:.1f}, dy: {kite.dY:.1f}",
                (10, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)
    cv2.putText(frame, f"Man x: {kite.x:.1f}, y: {kite.y:.1f}",
                (180, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    cv2.putText(frame, "Act Angle: " + str(int(kite.kiteangle)),
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, "Tgt Angle: " + str(int(kite.targetangle)), (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, "Tgt Heading: " + str(int(kite.targetheading)), (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, "Mode: " + str(control.config), (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, "Area: " + str(kite.contourarea), (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    if use_ros2 and len(joyaxes) > 2:
        cv2.putText(frame, "joy:" + str(joyaxes[2]), (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, "Counter:" + str(counter), (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return


def display_flight(screenwidth):
    # output flight values
    outx = screenwidth - 180
    fontsize = 0.5
    tempstr = "Found: Yes" if kite.found else "Found: No"
    cv2.putText(frame, 'Zone: ' + kite.zone, (outx, 40), cv2.FONT_HERSHEY_SIMPLEX, fontsize, (0, 0, 255), 2)
    cv2.putText(frame, tempstr, (outx, 60), cv2.FONT_HERSHEY_SIMPLEX, fontsize, (0, 0, 255), 2)
    cv2.putText(frame, 'Mode: ' + kite.mode, (outx, 80), cv2.FONT_HERSHEY_SIMPLEX, fontsize, (0, 0, 255), 2)
    cv2.putText(frame, 'Phase: ' + kite.phase, (outx, 100), cv2.FONT_HERSHEY_SIMPLEX, fontsize, (0, 0, 255), 2)
    return


def display_line(angle, cx, cy, radius, colour):
    pointx, pointy = get_angled_corners(cx + radius, cy, angle, cx, cy)
    pointx = int(pointx)
    pointy = int(pointy)
    offx = cx - (pointx - cx)
    offy = cy - (pointy - cy)
    cv2.line(frame, (offx, offy), (pointx, pointy), colour, 2)
    return


def display_motor_msg(action, setup):
    outx = width - 250
    outy = height - 30
    fontsize = 0.5
    cv2.putText(frame, 'Motor Msg: ' + str(action) + ' ' + setup, (outx, outy), cv2.FONT_HERSHEY_SIMPLEX, fontsize,
                (0, 255, 255), 2)
    return


def present_calibrate_row(row):
    # some sort of accuracy formulas here
    return f'Action: {row[0]} T_Time: {row[1]} A_Time: {row[2]} ' \
           f'T_Resist: {row[3]} A_Resist: {row[4]} {row[5]} Cycles: {row[6]} {row[7]} '


def mouse_events(event,x,y,flags,param):
    if(event == cv2.EVENT_LBUTTONDOWN):
        print('leftclick')


# MAIN ROUTINE START
parser = argparse.ArgumentParser()
parser.add_argument('-f', '--file', type=str, default='cachedH.npy',
                    help='Filename to load cached matrix')
parser.add_argument('-l', '--load', type=str, default='yes',
                    help='Do we load cached matrix')
parser.add_argument('-k', '--kite', type=str, default='Manbar',
                    help='Kite either Standard or Manual or Manbar')
parser.add_argument('-s', '--setup', type=str, default='Standard',
                    help='Standard, BarKiteActual, KiteBarInfer, KiteBarTarget')
# Standard means no connections between KiteAngle, KiteTargetAngle and Bar Angles others
# show connections from and to ie BarKiteActual the Kite angle is updated from the bar Angle
# This allows direct motor commands to be sent
parser.add_argument('-m', '--motortest', type=int, default=0, help='motortest either 0 or 1')
args = parser.parse_args()

# iphone
masklimit = 1000
# wind
# masklimit = 1000
# config = 'yellowballs'  # alternative when base not present will also possibly be combo
# KITETYPE = 'indoorkite'  # need to comment out for external
#KITETYPE = 'kite1'
KITETYPE = 'kite2'  # start for iphone SE video

# controls setup self.inputmodes = ('Standard', 'SetFlight', 'ManFly')
# setup options are Manfly, Standard

# initiate class instances
# config = Config(setup='Manfly', source=1, input='Joystick')
config = Config(source=2, kite=args.kite,  numcams=1, check_motor_sim=True, setup=args.setup, logging=1)
control = Controls(config.kite, step=16, motortest=args.motortest)
kite = Kite(300, 400, mode='fig8') if control.config == "Manual" else Kite(control.centrex, control.centrey, mode='fig8')
base = Base(kitebarratio=1, safety=True)
print('input', control.inputmode)

while config.source not in {1, 2}:
    config.source = input('Key 1 for camera or 2 for source')
# should define source here
if config.source == 1:
    # camera = cv2.VideoCapture(-1)
    # probably need to go below route to do stitching but need to understand differences first
    if config.numcams == 1:
        camera = VideoStream(src=-1).start()
        stitcher = None
    else:
        leftStream = VideoStream(src=2).start()  # think this is the top part to check
        rightStream = VideoStream(src=0).start()
        time.sleep(2.0)
        stitcher = Stitcher()
        if args.load == 'yes':
            try:
                stitcher.cachedH = np.load(args.file)
            except (FileNotFoundError, IOError):
                print("File not found continuing:", args.file)
    config.logging = 1
else:
    print('I am 2')
    # TODO at some point will change this to current directory and append file - not urgent
    # camera = cv2.VideoCapture(r'/home/donald/catkin_ws/src/kite_ros/scripts/choppedkite_horizshort.mp4')
    camera = cv2.VideoCapture(r'd:/dev/workspace/src/ros2_kite/ros2_kite/choppedkite_horizshort.mp4')
    #camera = cv2.VideoCapture(r'/home/ubuntu/catkin_ws/src/kite_ros/scripts/2020_test1.mp4')
    # Videostream seems to create errors with playback
    # camera = VideoStream(src=r'/home/donald/catkin_ws/src/kite_ros/scripts/choppedkite_horizshort.mp4').start()
    # camera =a VideoStream(src=r'/home/donald/catkin_ws/src/kite_ros/scripts/choppedkite_horizshort.mp4').start()
    # print('video:', camera.grab())
    config.logging = 1

# Initialisation steps
es = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
kernel = np.ones((5, 5), np.uint8)
background = None
#imagemessage = KiteImage()
if use_ros2:
    init_ros()
    init_motor_msg()

# def test_pid(P = 1.0,  I = 0.0, D= 0.0, L=100):
pid = PID.PID(1, 0, 0)
pid.setSampleTime(0.01)

# initialize the list of tracked points, the frame counter,
# and the coordinate deltas
counter = 0

if use_ros2:
    listen_kiteangle('kiteangle')  # this then updates base.barangle via the callback function
    if config.check_motor_sim:
        listen_kiteangle('mockangle')  # this then subscribes to our simulation of expected movement of the bar

    listen_joystick()  # subscribe to joystick messages - now only option
sg.theme('Black')  # Pysimplegui setup

# below is proving clunky if we may start with any mode as the buttons names get fixed here - so if keeping this logic
# we must always create buttons with std setup and then cycle to correct mode
initmodestring = control.getmodestring(0)  # now always get same initial modestring
button_list = initmodestring.split()[1:]
sgmodestring = 'Mode: ' + initmodestring.split()[0]
# horizontal buttons
# buttons = [sg.Button(x, size=(7, 3), pad=(4, 0), font='Helvetica 14') for i, x in enumerate(button_list)]
#vertical buttons
buttons = [[sg.Button(x, size=(7, 2), pad=(0, 1), font='Helvetica 14')] for i, x in enumerate(button_list)]
layout = [[sg.Text(sgmodestring, key=sgmodestring, size=(10, 1), font='Helvetica 20')], buttons]

# create the window and show it without the plot
window = sg.Window('Kite ROS - Automated Flying', layout, no_titlebar=False, location=(50, 50))
event, values = window.read(timeout=0)

if control.inputmode != 0:
    z = 0
    while z < control.inputmode:
        control.newbuttons = control.get_change_mode_buttons(z + 1)
        for x in control.newbuttons:  # change the button labels if mode has change
            window[x[0]].Update(x[1])
        z += 1

writer = None
cv2.startWindowThread()
cv2.namedWindow('contours')
cv2.setMouseCallback('contours', mouse_events)
fps = 15
# fps = camera.get(cv2.CV_CAP_PROP_FPS)

get_angles(kite, base, control, config)
if use_ros2:
    joybuttons, joyaxes = get_joystick()
time.sleep(2)
base.start_time = round(time.monotonic() * 1000)
writelogheader(config, kite, base, control)

# Main module loop START
while True:
    if config.numcams == 1:
        if config.source == 1:
            ret, frame = camera.stream.read()
        # change above for videostream from pyimagagesearch
        else:  # from file
            ret, frame = camera.read()
    else:
        left = leftStream.read()
        right = rightStream.read()
        # below is because opencv only stitches horizontally
        left = cv2.transpose(left)
        right = cv2.transpose(right)

        # resize the frames
        left = imutils.resize(left, width=480)
        right = imutils.resize(right, width=480)

        # stitch the frames together to form the panorama
        # IMPORTANT: you might have to change this line of code
        # depending on how your cameras are oriented; frames
        # should be supplied in left-to-right order
        result = stitcher.stitch([left, right])
        # reverse the transposition to get images back above one another
        camera = cv2.flip(cv2.transpose(result), 1)

        # no homography could be computed
        if camera is None:
            print("[INFO] homography could not be computed")
            break
        else:
            frame = camera
    #print('frame', frame.shape[1],frame.shape[0])
    height, width, channels = frame.shape
    writepictheader(config, height, width, fps)

    if background is None:
        background = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        background = cv2.GaussianBlur(background, (21, 21), 0)
        continue

    if not base.calibrate:
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_frame = cv2.GaussianBlur(gray_frame, (21, 21), 0)
        diff = cv2.absdiff(background, gray_frame)
        diff = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]
        diff = cv2.dilate(diff, es, iterations=2)
        if cv2.__version__.startswith('4'):
            image = None
            cnts, hierarchy = cv2.findContours(diff.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        elif cv2.__version__.startswith('3'):
            image, cnts, hierarchy = cv2.findContours(diff.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        else:
            raise AssertionError('cv2 must be either version 3 or 4 to call this method')

        # draw and move cross for manual flying
        if config.kite == 'Manual':
            drawkite(kite)
            kite.found = True
        elif config.kite == 'Standard':  # not detecting if in manual mode
            kite.found = False
            maxmask = -1
            index = -1
            for i, c in enumerate(cnts):
                mask = kitemask(c, frame, KITETYPE)
                if mask > maxmask:
                    index = i
                    maxmask = mask
                # (x, y, w, h) = cv2.boundingRect(c)
                # cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 125, 0), 2)

            if maxmask > masklimit:
                kite.found = True
                c = cnts[index]
                kite.contourarea = cv2.contourArea(cnts[index])
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 2)
                finalframe = frame[y:y + h, x:x + w]
                center = (x + (w // 2), y + (h // 2))
                kite.pts.appendleft(center)
                kite.x = center[0]
                kite.y = center[1]

                # Min Area seems reasonable to get angle of kite
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)  # cv2.boxPoints(rect) for OpenCV 3.x
                box = np.int0(box)
                cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
                kite.kiteangle = get_angle(box, kite.dX, kite.dY)

        # Establish route
        if kite.changezone or kite.changephase or kite.routechange:
            control.routepoints = calc_route(control.centrex, control.centrey, control.halfwidth, control.radius)
            kite.update_target(control.routepoints[0][0], control.routepoints[0][1],
                               control.centrex, control.maxy, control.routepoints[3][0], control.routepoints[3][1])
            kite.routechange = False
        # start direction and analysis - this will be a routine based on class
        getdirection(kite)
        kite.targetheading = get_heading_points((kite.x, kite.y), (kite.targetx, kite.targety))
        kite.update_zone(control)
        kite.update_phase()

    # Update actual angles based on full setup
    get_angles(kite, base, control, config)
    kite.targetangle = kite.targetheading

    if kite.zone == 'Centre' or kite.phase == 'Xwind':
        kite.targetangle = get_heading_points((kite.x, kite.y), (kite.targetx, kite.targety))

    drawroute(control.routepoints, control.centrex, control.centrey)
    drawcross(kite.targetx, kite.targety, 'Target', (0, 150, 250))
    if kite.autofly:
        kite.move_kite(control, 10)

    if use_ros2 and config.check_motor_sim:
        base.mockangle = get_actmockangle(kite, base, control, config)

    display_stats()
    display_flight(width)
    display_base(width)

    # kite_pos(kite.x, kite.y, kite.kiteangle, kite.dX, kite.dY, 0, 0)
    doaction = True if control.motortest or base.calibrate or control.inputmode == 3 else False

    if not doaction:
        pid.SetPoint = base.targetbarangle
        pid.update(base.barangle)
        base.action = get_action(pid.output, base.barangle)

    if use_ros2:
        motor_msg(base.action)
    display_motor_msg(base.action, config.setup)

    cv2.imshow("contours", frame)
    # below commented due to failing on 18.04
    # kiteimage.pubimage(imagemessage, frame)

    # read pysimplegui events
    event, values = window.read(timeout=0)
    for x in control.newbuttons:  # change the button labels if mode has change
        window[x[0]].Update(x[1])

    if use_ros2:
        joybuttons, joyaxes = get_joystick()
    else:
        joybuttons=None
        joyaxes=None
    cv2.imshow('contours', frame)

    if config.input == 'Keyboard':
        # change to -1 for debugging
        # 20 seems to work better than 1 on virtualbox - not sure what the issue is
        # will keep these separate again as the keyhandler interfered with joystick before hence
        # it got dropped - so now we will have mouse only below and keyboard + mouse where the intention is actually
        # to use additional mouse buttons as keys via x mouse control
        key = cv2.waitKey(20) & 0xff
        if key != -1:
            quitkey, resetH = control.keyhandler(key, kite, base, control, event)
    else:  # mouse only
        #quitkey, resetH = control.joyhandler(joybuttons, joyaxes, kite, base, control, event)
        quitkey, resetH = control.mousehandler(kite, base, control, event)


    #added cv2.waitKey back in for ubuntu 22.04 - not clear why it was neeed but window failed to display without it
    if quitkey or event in ('Quit', None) or cv2.waitKey(1) & 0xFF == ord('q'):  # quit if controls window closed or home key
        break

    if resetH and stitcher:
        stitcher.cachedH = None
    counter += 1
    countertup = (counter,)
    #writelogs(config, kite, base, control, frame, height, width, countertup)
    time.sleep(control.slow)

# Exit and clean up
print("[INFO] cleaning up...")
closelogs(config)
cv2.destroyAllWindows()
if config.numcams == 1:
    camera.stop()
else:
    leftStream.stop()
    rightStream.stop()

if writer is not None:
    writer.release()
