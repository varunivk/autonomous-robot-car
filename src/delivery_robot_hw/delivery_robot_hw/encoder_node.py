#!/usr/bin/env python3
"""
encoder_node.py
Counts pulses from the 4 TT hall-effect encoders, publishes wheel
angular speed (rad/s) on /wheel_speeds as [FL, RL, FR, RR].
FL->GPIO23, RL->GPIO24, FR->GPIO25, RR->GPIO27
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from gpiozero import Button

PULSES_PER_REV = 20   # calibrate: spin wheel exactly once, count pulses
PUBLISH_RATE_HZ = 10.0
PIN_FL, PIN_RL, PIN_FR, PIN_RR = 23, 24, 25, 27


class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        self.counts = {'fl': 0, 'rl': 0, 'fr': 0, 'rr': 0}

        self.enc_fl = Button(PIN_FL, pull_up=True)
        self.enc_rl = Button(PIN_RL, pull_up=True)
        self.enc_fr = Button(PIN_FR, pull_up=True)
        self.enc_rr = Button(PIN_RR, pull_up=True)

        self.enc_fl.when_pressed = lambda: self._count('fl')
        self.enc_rl.when_pressed = lambda: self._count('rl')
        self.enc_fr.when_pressed = lambda: self._count('fr')
        self.enc_rr.when_pressed = lambda: self._count('rr')

        self.pub = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_speeds)
        self.get_logger().info('encoder_node started, publishing /wheel_speeds')

    def _count(self, wheel):
        self.counts[wheel] += 1

    def publish_speeds(self):
        msg = Float32MultiArray()
        speeds = []
        for wheel in ('fl', 'rl', 'fr', 'rr'):
            pulses = self.counts[wheel]
            self.counts[wheel] = 0
            rev_per_sec = (pulses / PULSES_PER_REV) * PUBLISH_RATE_HZ
            speeds.append(rev_per_sec * 2 * math.pi)
        msg.data = speeds
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()