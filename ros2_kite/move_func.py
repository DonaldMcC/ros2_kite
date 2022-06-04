# This file contains functions mainly called by movingclass but possibly also by kitesimulate for
# conversion of stuff - fairly std functions I think and generally should now work with both
# 2 and 3D objects

#note these functions were originally written to work with pygame coordinates with origin at
# bottom left - this setup is now using opencv where origin is top left and I want 0 degrees
# to be straight up and 90 degrees being 3 o clock so some checking of this is required
# opencv_coords introduced to support this

import math
from utils import order_points
import numpy as np

opencv_coords = True

# set up the colors
# BLACK = (0, 0, 0)
# WHITE = (255, 255, 255)
# RED = (255, 0, 0)
# GREEN = (0, 255, 0)
# BLUE = (0, 0, 255)


def distance(xyz1, xyz2):
    """ returns the straight line distance between two tuples
    >>> distance((1, 0, 0), (0, 0, 0))
    1.0
    >>> distance((3, 4), (0, 0))
    5.0
    >>>
    """
    if len(xyz1) == 3 and len(xyz2) == 3:
        return math.sqrt((xyz1[0] - xyz2[0]) ** 2.0 + (xyz1[1] - xyz2[1]) ** 2.0 + (xyz1[2] - xyz2[2]) ** 2.0)
    else:
        return math.hypot((xyz1[0]-xyz2[0]), (xyz1[1]-xyz2[1]))


def get_angle(box, dx=0, dy=0, mindist=2):
    # orderbox is sorted tl, tr, br, bl
    """returns the angle of the kite right side lower is +ve and 0 is a parked kite
       this is perhaps confusing but stems from opencv having y axis 0 at the top

        Parked kite
        >>> get_angle(np.array([[11,9],[11,6],[15,6],[15,9]]),0,-20)
        (0.0, 0.0)
        >>> get_angle(np.array([[11,9],[11,5],[14,5],[14,9]]),10, 0)
        (90.0, 90.0)
        >>> get_angle(np.array([[11,9],[11,5],[14,5],[14,9]]),-10, 0)
        (-90.0, -90.0)
        >>> get_angle(np.array([[11,9],[11,5],[14,5],[14,9]]),-100, 0)
        (-90.0, -90.0)
        >>> get_angle(np.array([[11,8],[12,6],[15,8],[14,10]]),10, -30)
        (26.56505117707799, 18.43494882292201)
        >>> get_angle(np.array([[11,8],[12,6],[15,8],[14,10]]),-5, 30)
        (-153.43494882292202, -170.53767779197437)
        >>> get_angle(np.array([[8,9],[6,8],[8,5],[10,6]]),-30, -10)
        (-63.43494882292201, -71.56505117707799)
        >>> get_angle(np.array([[8,9],[6,8],[8,5],[10,6]]), 30, 10)
        (116.56505117707799, 108.43494882292202)

       """

    orderbox = order_points(box)
    # we want the angle of the short side of the rectangle
    # print distance(orderbox[3], orderbox[0])
    # print distance(orderbox[1], orderbox[0])

    if distance(orderbox[0], orderbox[1]) < distance(orderbox[1], orderbox[2]):
        # build unitvect for short side of the rectangle
        unitvect = heading(orderbox[0], orderbox[1])
    else:
        unitvect = heading(orderbox[3], orderbox[0])
    # print unitvect
    angle = get_heading(unitvect[0], unitvect[1])
    heading_angle = get_heading(dx, dy)

    if math.sqrt(dx*dx + dy*dy) > mindist:
        # if moving significantly then assume not going backwards as this is hard to do
        if abs(angle-heading_angle) > 90:
            if angle > 0:
                angle -= 180
            else:
                angle += 180

    if __name__ == '__main__':
        return angle, heading_angle
    else:
        return angle


def speed(d, t):
    try:
        s = float(d)/t
        s *= 1000.0
    except ZeroDivisionError:
        s = 0
    return s


def heading(xyz1, xyz2):
    """ returns a unit vector for the heading from vector1 to vector2
       >>> heading ((2,2),(3,3))
       (0.7071067811865475, 0.7071067811865475)
       >>>
    """
    d = distance(xyz1, xyz2)
    if len(xyz1) == 3 and len(xyz2) == 3:
        try:
            x = (xyz2[0]-xyz1[0])/d
            y = (xyz2[1]-xyz1[1])/d
            z = (xyz2[2]-xyz1[2])/d
        except ZeroDivisionError:
            x, y, z = 0, 0, 0
        return x, y, z
    else:
        try:
            x = (xyz2[0]-xyz1[0])/d
            y = (xyz2[1]-xyz1[1])/d
            z = 0
        except ZeroDivisionError:
            x, y, z = 0, 0, 0
    return x, y


def vector_mult(vector, mult):
    """ Multiply a vector by a value
       >>> vector_mult((2,3),4)
       (8, 12)
       >>>
    """
    return tuple([l * mult for l in vector])


def vector_add(vector1, vector2):
    """ Add two vectors togetther
       >>> vector_add((2, 3), (4, 6))
       (6, 9)
       >>>
    """
    if len(vector1)<3 or len(vector2)<3:
        return vector1[0] + vector2[0], vector1[1] + vector2[1]
    else:
        return vector1[0] + vector2[0], vector1[1] + vector2[1], vector1[2] + vector2[2]


def rotate90(x: int, y: int, dir='Clockwise') -> tuple:
    """
    This rotates a 2d vector 90 degrees either clockwise or anti-clockwise
       >>> rotate90(2, 1)
       (1, -2)
       >>> rotate90(2,1,'Anti')
       (-1, 2)
       >>>
    """

    if dir == 'Clockwise':
        return y, -x
    else:
        return -y, x


def rotate3d(point, angle, axis='y'):
    """ This rotates an xyz tuple by angle degrees around an axis by default y
       >>> rotate3d((10, 10, 10), 45)
       (8.881784197001252e-16, 10, 14.142135623730951)
       >>>
    """

    if len(point) != 3:
        return 'error'
    ang_rad = math.radians(angle)
    x = point[0] * math.cos(ang_rad) - point[2] * math.sin(ang_rad)
    y = point[1]
    z = point[0] * math.sin(ang_rad) + point[2] * math.cos(ang_rad)

    return x, y, z
    

def get_heading(x: any, y: any) -> float:
    """Calcs angle in degrees based on single point and origin
       >>> get_heading(0, -1)
       0.0
       >>> get_heading(1.0, -2.0)
       26.56505117707799
       >>> get_heading(-30, -15)
       -63.43494882292201
       >>> get_heading(1, -1)
       45.0
       >>>
       """
    if opencv_coords:
        return math.degrees(math.atan2(x, -y))
    else:
        return math.degrees(math.atan2(x, y))

    
def get_heading_points(pt1, pt2) -> float:
    """Calcs the angle between 2 points
    >>> get_heading_points((3,3), (4,2))
    45.0
    """
    x, y = heading(pt1, pt2)
    return get_heading(x, y)


def get_coord(x, y, anglechange):
    # This calculates, probably badly, the angle of a unit vector in degrees
    # and then updates based on the anglechange and
    # returns the new coordinates based on the x axis being
    # set to 0 degrees (may want to change this to make N the top of the screen

    if x >= 0 and y >= 0:
        angle = math.degrees(math.asin(y))
    elif x >= 0 > y:
        angle = 360 + math.degrees(math.asin(y))
    elif x < 0 <= y:
        angle = math.degrees(math.acos(x))
    else:  
        angle = 360 - math.degrees(math.acos(x))

    newangle = angle + anglechange
    if newangle > 360:
        newangle -= 360
    elif newangle < 0:
        newangle += 360

    newangle = math.radians(newangle)
    
    newx = math.cos(newangle)
    newy = math.sin(newangle)
    return newx, newy


def get_angled_corners(x: int, y: int, anglechange: int, centx=0, centy=0, format='float') -> tuple:
    """
    This calculates the new co-ordinates of a point on a circle with centre centx, centy
    when rotated through anglechange degrees
    This was stuff for points on a circlex = 30
    y = 40
    r = 25
    for i in range(0,20):
    a = (i/10.0) * math.pi
    cx = x + r * math.cos(a)
    cy = y + r * math.sin(a)
    print(cx,cy)

    >>> get_angled_corners(3,4,180)
    (-2.9999999999999987, -4.000000000000001)
    >>> get_angled_corners(3,-4,180)
    (-3.0000000000000004, 3.9999999999999996)
    >>> get_angled_corners(-3,4,180)
    (3.0, -4.0)
    >>> get_angled_corners(-3,-4,180)
    (2.9999999999999987, 4.000000000000002)
    >>> get_angled_corners(1,0,90)
    (0, -1)
    >>>
    """

    anglechange = -anglechange  #  swapped for different axis in opencv
    x = x - centx
    y = y - centy
    
    radius = math.sqrt(x * x + y * y)
    rads = math.radians(anglechange)

    if x >= 0 and y >= 0:
        angle = math.asin(x/radius)
    elif x >= 0 > y:
        angle = 0.5 * math.pi + math.acos(x / radius)
    elif x < 0 <= y:
        angle = 0 + math.asin(x/radius)
    else:  # x<0 and y<0
        angle = math.pi + math.asin(-x / radius)

    newangle = angle + rads
    # print(newangle * 180/math.pi)
    cx = radius * math.sin(newangle) + centx
    cy = radius * math.cos(newangle) + centy
    if format == 'int':
        cx = int(cx)
        cy = int(cy)
    return cx, cy


def get_corners(x, y, width, height, shape='rectangle', bottom=0, angle=0) -> tuple:
    """
        This takes a centre point and calculates the corresponding corners of a rectangle of given width and height
       optional parameters are to return a kite based diamond in which case the bottom would typically be longer
       than the top and finally if you need to tilt the shape then you can specify the angle to rotate the points
       by in degrees with 0 meaning no tilt and others rotating all points.
    >>> get_corners(30, 40, 10, 20)
    ((25, 30), (25, 50), (35, 50), (35, 30))
    >>> get_corners(30, 40, 20, 10, 'kite', 30)
    ((20, 40), (30, 50), (40, 40), (30, 10))
    >>> get_corners(30, 40, 10, 20, 'rectangle', 0,30)
    ((20.669872981077805, 33.83974596215562), (30.669872981077805, 51.16025403784439), (39.33012701892219, 46.16025403784439), (29.330127018922198, 28.83974596215561))
    >>>
    """
    if shape == 'rectangle':
        corners = ((x - (width / 2), y - (height / 2)),
                   (x - (width / 2), y + (height / 2)),
                   (x + (width / 2), y + (height / 2)),
                   (x + (width / 2), y - (height / 2)))
    elif shape == 'kite':
        # kite shape
        # print(x,y,width,height)
        corners = ((x - (width / 2), y),
                   (x, y + height),
                   (x + (width / 2), y),
                   (x, y - bottom))

    if angle == 0:
        return corners
    else:
        # need to do some sort of mapping to return - probably just use GetCoord
        angledcorners = tuple([get_angled_corners(pt[0], pt[1], angle, x, y) for pt in corners])
        return angledcorners


def get_coord_sphere(long, lat, r) -> tuple:
    """ Let r = radius of a sphere, omega = longitude angle, and phi = latitude angle.
       Omega = 0 at London's longitude. phi = 0 at the equator.
       r varies from r = 0 to R Earth's radius to R + h where h is the height above the surface of Earth. 

       Then x = r cos(phi)cos(omega),
            z = r cos(phi)sin(omega),
        and y = r sin(phi)

        >>> get_coord_sphere(0,90,20)
        (0.0, 20.0, 1.2246467991473533e-15)
        >>> get_coord_sphere(45,45,20)
        (10.0, 14.14213562373095, 10.000000000000002)
        >>> get_coord_sphere(-80,20,20)
        (-18.50833156796647, 6.840402866513374, 3.2635182233306983)
        >>>

    """

    omega = math.radians(long)
    phi = math.radians(lat)
    z = r * math.cos(phi) * math.cos(omega)
    x = r * math.cos(phi) * math.sin(omega)
    y = r * math.sin(phi)
    
    return x, y, z


def get_long_lat(x, y, z):
    """This should be the inverse of get_coord sphere
       latitude = asin (y/R) and longitude = atan2 (z,x).
    >>> get_long_lat(-18.50833156796647, 6.840402866513374, 3.2635182233306983)
    (-80.0, 20.0, 20.0)
    >>> get_long_lat(0.0, 20.0, 0)
    (0.0, 90.0, 20.0)
    >>> get_long_lat(5.0, 27.0, 11)
    (24.443954780416536, 65.89051882013823, 29.58039891549808)
    >>> get_long_lat(10.0, 14.14213562373095, 10.000000000000002)
    (44.99999999999999, 44.99999999999999, 20.0)
    >>>
    """

    r = math.sqrt(x ** 2 + y ** 2 + z ** 2)
    try:
        lat = math.degrees(math.asin(y / r))
        long = math.degrees(math.atan2(x, z))
        if long > 90:
            long = (long - 90) * -1
    except ZeroDivisionError:
        long = 0
        lat = 0
    return long, lat, r


def conv_lin(a, b=1.0, c=0.0, inverse=False):
    """Simple linear transform
        will I think store parameters against each sensor then they are handy
        >>> conv_lin(4,2,3)
        11
        >>> conv_lin(11,2,3.0,True)
        4.0
        >>>"""
    if inverse is False:
        return a * b + c
    else:
        return (a - c) / b


def adjust_line_length(x:int, y:int, z:int, r:int) -> tuple:
    """
        This adjusts kite line length based to ensure kite is at line length
       and is required because we are assuming straight line motion so x,y,z
       input to this function probably won't have radius r but the nearest point will
       be a projection through the sphere if x,y,z is outwith
       so all we need to do is get the unit vector for the heading
       x,y,z and then multiply by r

       >>> adjust_line_length(6, 8, 3, 10)
       (5.746957711326908, 7.662610281769211, 2.873478855663454)
       >>> adjust_line_length(5, 8, 3, 10)
       (5.050762722761054, 8.081220356417685, 3.0304576336566322)
    """

    unitv = heading((0, 0, 0), (x, y, z))
    finalv = vector_mult(unitv, r)
    return finalv[0], finalv[1], finalv[2]

    
def get_plan_pos(p:int, h:int, t:int, s:int) ->tuple:
    """
    Return the position of an object in position p on heading h (unit vector after time t if travelling at speed s
       >>> get_plan_pos((1, 2, 3), (0, 0.707,0.707), 1, 5)
       (1, 5.535, 6.535)
    """
    mvmt = vector_mult(h, t*s)
    return vector_add(p, mvmt)


def move_item(x:int, y:int, targetx:int, targety:int, distance:int) -> tuple:
    """
        >>> move_item(1, 1, 4, 5, 5)
        (4.0, 5.0)
        >>> move_item(0, 0, 2, 2, 4)
        (2.82842712474619, 2.82842712474619)
        """
    # approach is work out distance between
    # use the distance function and then we have a ratio of how much of the distance we got
    # then we do vectormult and that gives us the new point which we add to the start??
    myheading = heading((x, y), (targetx, targety))
    return get_plan_pos((x,y), myheading, 1, distance)


def _test():
    import doctest
    doctest.testmod()
    

if __name__ == '__main__':
    'Can run with -v option if you want to confirm tests were run'
    _test()
