#!/usr/bin/env python
# this runs as separate ROS node and publishes messages from joystick - generally working fin
# to delet as ROS no longer part of this project - just going with opencv and arduino di
import rclpy
from sensor_msgs.msg import Joy
joybuttons = []
joyaxes = []


def listen_joystick():
    rclpy.Subscriber('xwiimote_node/joy', Joy, callback)


def callback(data):
    global joybuttons, joyaxes
    joybuttons = data.buttons
    joyaxes = data.axes
    return


def get_joystick():
    return joybuttons, joyaxes


if __name__ == '__main__':
    rclpy.init_node('joy_listen', anonymous=False)
    listen_joystick()
    rclpy.spin()
