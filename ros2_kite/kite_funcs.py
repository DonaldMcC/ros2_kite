import numpy as np
import cv2

# resistance = 200#

# this should always return barangle except when barangle being set from the kite for simulation
# or on manbar when bar should be freely controlled
def get_barangle(kite, base, control, config):
    if config.setup == 'KiteBarActual':
        return kite.kiteangle / base.kitebarratio
    else:  # automated flight reading from some sort of sensor via ROS
        barangle = getangle(base.resistance, base.maxleft, base.maxright,
                            base.resistleft, base.resistright, base.resistcentre)
        return barangle


# this is not currently used - but may need again for simulation
def get_actmockangle(kite, base, control, config):
    # global mockangle, mockresistance
    mockangle = getangle(base.resistance, base.maxleft, base.maxright,
                         base.resistleft, base.resistright, base.resistcentre)
    return mockangle

# http://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
# define the list of boundaries
# boundaries = [([0, 0, 0], [40, 40, 40])]
# green
# boundaries = [([10, 100, 10], [100, 255, 100])]
# orange
# boundaries = [([0, 50, 100], [100, 200, 255])]

# iphone video
# contourmin = 3000
contourmin = 800


def kitemask(c, frame, kitecolours='kite1'):
    # This sets the properties for the kite we are looking for
    # setup for now is just for kite1 but we can be looking for in
    # different conditions and this might affect the colours
    # so think we amend this to add the object for indoorkite
    # and 
    if cv2.contourArea(c) < contourmin:
        return 0
    if kitecolours == 'indoorkite':
        boundaries = [([10, 10, 140], [70, 70, 200])]
    elif kitecolours == 'kite1':
        boundaries = [([0, 0, 100], [100, 100, 255]),
                      ([0, 50, 100], [120, 220, 255])
                      ]
    else:   # 'kite2'
        boundaries = [([0, 0, 0], [30, 30, 30]),
                      ([10, 10, 100], [100, 100, 255]),
                      ([0, 50, 100], [120, 220, 255])
                      ]
        # iphone
        boundaries = [([0, 0, 100], [100, 100, 255]),
                      ([0, 50, 150], [120, 220, 255])
                      ]
    totmask = 1
    for (lower, upper) in boundaries:
        # create NumPy arrays from the boundaries
        low = np.array(lower, dtype="uint8")
        upp = np.array(upper, dtype="uint8")

        (x, y, w, h) = cv2.boundingRect(c)
        roi = frame[y:y + h, x:x + w]
        # loop over the boundaries
        mask = cv2.inRange(roi, low, upp)
        totmask *= np.sum(mask)
        # print(x, y, w, h, "cont", cv2.contourArea(c))
        # print("mask: ", np.sum(mask), totmask)
    return totmask


def checklimits(angle, maxleft, maxright):
    """
    :param angle:
    :param maxleft:
    :param maxright:
    :return:

    >>> checklimits(50, -45, 30)
    30
    >>> checklimits(-50, -45, 30)
    -45
    >>> checklimits(-20, -45, 30)
    -20
    """

    angle = max(angle, maxleft)
    angle = min(angle, maxright)
    return angle


def getangle(resistance, maxleft=0, maxright=9999, resistleft=9999, resistright=99999, resistcentre=0):
    """
    :param resistcentre:
    :param resistright:
    :param resistleft:
    :param maxright:
    :param maxleft:
    :param resistance:
    :return angle:

    >>> getangle(267)
    20
    >>> getangle(200)
    0
    >>> getangle(110)
    -30
    >>> getangle(155)
    -15
    """

    # calibration is based on 0 being the centre and maxleft and maxright being
    # defined in degrees - the corrsesponding values of the resistor should be taken
    # for all of these and we will for now assume resistor is linear - have now changed
    # so that values beyond maxleft and maxright should be supported
    angle = 0
    if resistance > resistcentre:
        angle = ((resistance - resistcentre) * maxright) / (resistright - resistcentre)
    elif resistance < resistcentre:
        angle = ((resistance - resistcentre) * maxleft) / (resistleft - resistcentre)

    return int(angle)


# TODO - think this should always be called with actual values
def getresist(angle, maxleft=0, maxright=9999, resistleft=9999, resistright=99999, resistcentre=0):
    """
    :param resistcentre:
    :param resistright:
    :param resistleft:
    :param maxright:
    :param maxleft:
    :param angle:
    :return angle:

    >>> getresist(-30)
    110
    >>> getresist(-15)
    155
    >>> getresist(0)
    200
    >>> getresist(20)
    267
    >>> getresist(10)
    233
    """

    # calibration is based on 0 being the centre and maxleft and maxright being
    # defined in degrees - the corrsesponding values of the resistor should be taken
    # for all of these and we will for now assume resistor is linear

    if angle < 0:
        resistance = resistleft + ((angle - maxleft) * (resistcentre - resistleft) / (0 - maxleft))
    elif angle > 0:
        resistance = resistright + ((maxright - angle) * (resistcentre - resistright) / maxright)
    else:
        resistance = resistcentre
    return int(resistance)


def get_action(output, barangle, maxleft=0, maxright=9999):
    # Now added ability to send motor message 6 for leftonly and 7 for rightonly
    # and will now add speed into the message as %age of max value up to 99 but 0 is max speed
    """
        :param output:
        :param barangle:
        :return action:

        >>> get_action(-10, -5)
        300
        >>> get_action(0, 0)
        0
        """
    TOLERANCE = 1  # degree of tolerance

    action = 0
    if abs(output) < TOLERANCE:
        action = 0  # stop
    elif output < 0 and barangle > maxleft:
        action = 300  # Left
    elif output > 0 and barangle < maxright:
        action = 400  # Right

    # TODO think about how PID impacts this if at all - speed should prob be used
    # action = int(msg + speed) if 0 < speed < 100 else int(msg)
    return action


def get_angles(kite, base, control, config):
    # base.resistance = resistance
    base.barangle = get_barangle(kite, base, control, config)
    # print('setr to ' + str(resistance))
    if config.setup == 'KiteBarTarget':
        base.targetbarangle = kite.kiteangle / base.kitebarratio
    else:
        base.targetbarangle = calcbarangle(kite, base, control)
    if config.setup == 'BarKiteActual':  # derive kite from bar
        kite.kiteangle = base.barangle * base.kitebarratio
    elif config.setup == 'KiteBarInfer':
        base.inferbarangle = inferangle(kite, base, control)
    return


def calcbarangle(kite, base, controls):
    """This should just basically set the target bar angle based on the mode phase
    and zone we are in when in park or wiggle mode
    >>> k=Kite(400, targetangle=10)
    >>> b=Base(barangle=15, kitebarratio=2)
    >>> c=Controls(1)
    >>> calcbarangle(k, b, c)
    10

    >>> k=Kite(400, phase='TurnR', targetangle=10)
    >>> b=Base(barangle=15, kitebarratio=2)
    >>> c=Controls(1)
    >>> calcbarangle(k, b, c)
    30
    """

    if kite.phase == "TurnR" or kite.phase == "TurnL":
        return setangleturn(kite, base)
    else:
        return setangle(kite, base, controls)


def inferangle(kite, base, controls=None):
    """This will return inferred angle of the control bar based on the  kite angle
       obviously this is a huge approximation of reality but should help with review
       of actual changes of the bar versus what we would like to happen ie no lag when
       testing

        >>> k=Kite(400, kiteangle=10)
        >>> b=Base(barangle=15, kitebarratio=2.0)
        >>> c=Controls(1)
        >>> inferangle(k, b, c)
        5.0
        """
    return kite.kiteangle / base.kitebarratio


def setangle(kite, base, controls):
    """This will return targetbarangle for park mode based largely on kite target angle
    We will start simple but may move to a pid mode if required

    >>> k=Kite(400, targetangle=10)
    >>> b=Base(barangle=15, kitebarratio=2)
    >>> c=Controls(1)
    >>> setangle(k,b,c)
    10
    """

    # targetbarangle = checklimits((kite.targetangle * base.kitebarratio), base.maxleft, base.maxright)
    targetbarangle = checklimits(kite.targetangle, base.maxleft, base.maxright)
    return targetbarangle


def setangleturn(kite, base):
    """This should be a simple function as we will always aim to turn as fast as poss
    identifying the point to ease off from max turn should be done as part of phase setting and not here

    >>> k=Kite(400)
    >>> b=Base(400)
    >>> setangleturn(k,b)
    -40
    """

    targetbarangle = base.maxright if kite.phase == "TurnR" else base.maxleft
    return targetbarangle


def calc_route(centrex=400, centrey=300, halfwidth=200, radius=100):
    """This just calculates the 6 points in our basic figure of eight
    should be easy enough and we then draw lines between each point and
    get the last point

    >>> calc_route(400, 300, 200, 100)
    [(200, 400), (100, 300), (200, 200), (600, 400), (700, 300), (600, 200)]

    """
    leftx = centrex - halfwidth
    rightx = centrex + halfwidth
    pt0 = (leftx, centrey + radius)
    pt1 = (leftx - radius, centrey)
    pt2 = (leftx, centrey - radius)
    pt3 = (rightx, centrey + radius)
    pt4 = (rightx + radius, centrey)
    pt5 = (rightx, centrey - radius)
    return [pt0, pt1, pt2, pt3, pt4, pt5]


def _test():
    import doctest
    doctest.testmod(verbose=False)


if __name__ == '__main__':
    'Can run with -v option if you want to confirm tests were run'
    from kite_class import Kite
    from base_class import Base
    from config_class import Config
    from control_class import Controls
    _test()
