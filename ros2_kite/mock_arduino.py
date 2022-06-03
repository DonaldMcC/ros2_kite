#!/usr/bin/env python
# this should generally receive motor msg which is currently just left or right based on 3 or 4 respectively
# or stop at 0.  1 and 2 are forward and backwards for initialisation - the purpose of this was initially to allow
# test of the setup without any actual arduino hardware and that should be provided by the kiteangle function
# testkiteangle is similar and possibly now redundant but both were receiving motormsg published by bascic_motion_det.

# What we now also want to do is verify that our simulation and actual operation are aligned this seems to need a
# different approach in two ways
# 1 We generate the motor_msg more simply - and possibly without the analysis
# 2 the mock_arduino process returns a different message from the actual one and we can ideally compare them and
# 3 understand if performance is as expected

"""Our setup is currently 2 actuators that should move in opposite directions and a central guide rail
for a wooden bar which will hold the kite by means of putting velcro on the handles - so basically just a
lever with short distance to the actuator - using ones designed for automatic doors and longer distance
to the kite handle.

dist_act will be from fulcrum to actuator
dist_handle will be from fulcrum to kite handle
speed_act is speed of actuator in mm per second 30mm/sec is current setup
force_act is max force of actuator (without leverage) 200N in my case
speed_handle is speed of handle in meters per second which I would like to be at least 30cm per sec as and lets
aim for 35cm of dist_handle and 3.5cm of dist_act as starting point as seems to make the maths easy so got 2 * 200N
of force which I think is enough as we are only changing the bar angle the main kite pull force should still be on
the frame

So above is simple arithmetic but bar_angle is slightly more tricky as dist_act will be constrained to an arc and
also got problem of not breaking lever which makes wider spacing tempting but I think too slow - probably we can
aim to stop at certain bar angles but this relies on sensor forces presumably change a bit outside centre of arc
but I think work on centre span needs first

So issue now is we are operating with two different parameter sets on converting resistance to angle because mock
arduino doesn't have our actual parameters but we need to make the defaults the same and that should avoid most of the
oscilation that has been going on
"""

# https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
# is basis for below

# and runs with ros2 run ros2_kite mock_ard
import time
import math
import rclpy
from rclpy.node import Node
import sys
#sys.path.append("/path/to/install/planner_pkg/lib/python3.6/site-packages/planner_pkg")

import argparse
from std_msgs.msg import String, Int16
from ros2_kite.kite_funcs import getresist, conmaxleft, conmaxright

# from kite_funcs import checklimits, getresist, conmaxleft, conmaxright, conresistleft,\
#     conresistright, conresistcentre
MAXLEFT = conmaxleft  # These are to simulate limits of angles
MAXRIGHT = conmaxright  # similarly to protect bar as attached close to pivot

motorvalue = 500  # stop
barangle = 0
DIST_ACT = 70.0  # mm # this is horizontal distance between centre and bar - doubled to reflect curr setup
DIST_HANDLE = 350.0  # mm
SPEED_ACT = 30.0  # mm/sec
FORCE_ACT = 200  # N but not sure if will actually use this
CIRC_ACT = 2 * math.pi * DIST_ACT

class MinimalPublisher(Node):
    def __init__(self):
        #super().__init__('minimal_publisher')
        super().__init__('mock_arduino')
        self.publisher_ = self.create_publisher(Int16, 'topic', 10)
        timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer = self.create_timer(timer_period, self.mock_kiteangle)
        self.i = 0
        self.cycle_time = 0.5

    def timer_callback(self):
        msg2 = String()
        msg2.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg2)
        self.get_logger().info('Publishing: "%s"' % msg2.data)
        self.i += 1

    def mock_kiteangle(self):
        msg = Int16()
        global motorvalue
        global barangle
        #pub = rclpy.Publisher(message, Int16, queue_size=1)
        #rclpy.init_node('mock_arduino', anonymous=False)
        #rate = rclpy.Rate(20)  # Cycles per Second
        # left_act_pos = get_coord(0-DIST_ACT, 0, barangle) not convinced this serves purpose

        #listen_motormsg()

        get_motorv()
        print('motorv', motorvalue)
        #cycle_time = round(time.monotonic() * 1000) - loop_time
        #loop_time = round(time.monotonic() * 1000)
        barangle = mockangle(barangle, self.cycle_time)
        resistance = getresist(barangle)
        msg.data=resistance
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        print(self.cycle_time, barangle, resistance)
        self.i += 1


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String,'topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def not_main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()



def mockangle(angle, elapsed_time):
    """This now attempts to simulate how we believe the bar should respond to messages sent to
    the actuator given known distance from 'fulcrum' to mounting points and speed of the actuator.
    Motorvalue is received for left and right and resistance is sent back as kiteangle message."""
    speed = 255

    global motorvalue
    get_motorv()
    direction = motorvalue // 100
    rawspeed = motorvalue % 100
    speed = int((rawspeed * 255)/100) if rawspeed > 0 else 255
    if direction == 1 or direction == 2:
        angle = 0
    else:
        if direction == 3:  # Left
            act_dist = 0 - (speed * SPEED_ACT * elapsed_time / (1000.0 * 255))
        elif direction == 4:  # Right
            act_dist = speed * SPEED_ACT * elapsed_time / (1000.0 * 255)
        elif direction == 6:  # Left Motor Only ie going right
            act_dist = speed * SPEED_ACT * elapsed_time / (2000.0 * 255)
        elif direction == 7:  # Right Motor only ie going left
            act_dist = 0 - (speed * SPEED_ACT * elapsed_time / (2000.0 * 255))
        else:
            act_dist = 0
        angle += (360 * act_dist) / CIRC_ACT
    if angle <= MAXLEFT:
        angle = MAXLEFT
    elif angle >= MAXRIGHT:
        angle = MAXRIGHT
    return angle


def listen_motormsg():
    rclpy.Subscriber('motormsg', Int16, callback, queue_size=1)


def callback(data):
    global motorvalue
    motorvalue = data.data
    return


def get_motorv():
    global motorvalue
    return motorvalue


def main(args=None):
    """
    :param args:
    :return:
    """
    rclpy.init(args=args)
    mock_arduino=MinimalPublisher()
    rclpy.spin(mock_arduino)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mock_arduino.destroy_node()
    rclpy.shutdown()
    return


if __name__ == '__main__':
    main()

"""
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('-m', '--message', type=str, default='kiteangle',
                            help='message to generate either kiteangle or mockangle')
        args = parser.parse_args()
        new_angle = mock_kiteangle(args.message)
    except rclpy.ROSInterruptException:
        pass
"""