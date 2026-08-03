#!/usr/bin/env python3
"""
motor_driver_node.py
Converts /cmd_vel into left/right skid-steer commands via gpiozero.
ENA/ENB jumpered to 5V on both L298N boards (always enabled).
Left  IN1->GPIO5, IN2->GPIO6 | Right IN1->GPIO16, IN2->GPIO20
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor

WHEEL_TRACK = 0.18       # meters, measure your actual chassis
MAX_LINEAR_SPEED = 0.45  # m/s cap, matches sim DiffDrive

LEFT_FWD, LEFT_BWD = 5, 6
RIGHT_FWD, RIGHT_BWD = 16, 20


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.left_motor = Motor(forward=LEFT_FWD, backward=LEFT_BWD, pwm=True)
        self.right_motor = Motor(forward=RIGHT_FWD, backward=RIGHT_BWD, pwm=True)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.2, self.watchdog_check)
        self.get_logger().info('motor_driver_node started, listening on /cmd_vel')

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        linear, angular = msg.linear.x, msg.angular.z

        v_left = linear - (angular * WHEEL_TRACK / 2.0)
        v_right = linear + (angular * WHEEL_TRACK / 2.0)

        left_speed = max(-1.0, min(1.0, v_left / MAX_LINEAR_SPEED))
        right_speed = max(-1.0, min(1.0, v_right / MAX_LINEAR_SPEED))

        self.set_motor(self.left_motor, left_speed)
        self.set_motor(self.right_motor, right_speed)

    def set_motor(self, motor: Motor, speed: float):
        if speed > 0.02:
            motor.forward(speed)
        elif speed < -0.02:
            motor.backward(-speed)
        else:
            motor.stop()

    def watchdog_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:  # no cmd_vel in 0.5s -> stop, safety
            self.left_motor.stop()
            self.right_motor.stop()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.left_motor.stop()
        node.right_motor.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()