#!/usr/bin/env python3
# from ros wiki for initial testing
# this will also go once arduino up and running

import rclpy
from rclpy.node import Node
from time import sleep
from std_msgs.msg import String, Int16
pub = 0


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('motormsg')
        self.publisher_ = self.create_publisher(Int16, 'topic', 10)
        timer_period = 0.1  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer = self.create_timer(timer_period, self.pub_motor)
        self.i = 0
        self.cycle_time = 0.5

    def timer_callback(self):
        msg2 = String()
        msg2.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg2)
        self.get_logger().info('Publishing: "%s"' % msg2.data)
        self.i += 1

    def pub_motor(self, motorvalue=666):
        msg = Int16()
        msg.data=motorvalue
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    """
    :param args:
    :return:
    """
    rclpy.init(args=args)
    motor_msg = MinimalPublisher()
    rclpy.spin(motor_msg)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_msg.destroy_node()
    rclpy.shutdown()
    return


if __name__ == '__main__':
    main()