#!/usr/bin/env python3
"""
camera_node.py
Captures frames from a USB webcam (via OpenCV/V4L2) and publishes them
on /camera/image as sensor_msgs/Image, matching the topic contract the
sim stack (camera_navigator, dashboard_node) already expects.

Tuned for: Quantron Mr. Boss QPC-1010 (480p hardware, 30fps, UVC plug-and-play)

NOTE: A plain USB webcam has no depth sensor. This node only replaces
the front RGB feed (/camera/image). It does NOT publish
/camera/depth_image or any of the /camera/{left,right,rear}/* topics
that camera_navigator relies on for obstacle avoidance. Until those
are provided (stereo pair, depth camera, or ultrasonic-derived
clearance), camera_navigator will sit in CAMERA_LOST on real hardware.
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

DEFAULT_DEVICE = '/dev/video0'
# Quantron Mr. Boss QPC-1010: real hardware resolution is 480p @ 30fps.
# The "30 MP" / "25 MP" marketing figures are software interpolation on
# the Windows driver UI only — request 640x480 directly, don't rely on it.
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 30.0
DEFAULT_FRAME_ID = 'camera_optical_frame'  # matches urdf.xacro camera_optical_frame


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('video_device', DEFAULT_DEVICE)
        self.declare_parameter('frame_width', DEFAULT_WIDTH)
        self.declare_parameter('frame_height', DEFAULT_HEIGHT)
        self.declare_parameter('fps', DEFAULT_FPS)
        self.declare_parameter('frame_id', DEFAULT_FRAME_ID)
        self.declare_parameter('flip_horizontal', False)
        self.declare_parameter('flip_vertical', False)

        device = self.get_parameter('video_device').value
        width = int(self.get_parameter('frame_width').value)
        height = int(self.get_parameter('frame_height').value)
        fps = float(self.get_parameter('fps').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.flip_h = bool(self.get_parameter('flip_horizontal').value)
        self.flip_v = bool(self.get_parameter('flip_vertical').value)

        self.bridge = CvBridge()

        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.get_logger().error(
                f'Could not open video device {device}. '
                'Check `ls /dev/video*` and permissions (user in "video" group).'
            )
            raise RuntimeError(f'Failed to open camera device: {device}')

        # Cheap UVC cams (this one included) often boot into a YUYV mode
        # that caps FPS well below 30 at 480p over USB2. Requesting MJPG
        # first lets the sensor's compressed mode actually hit 30fps.
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)

        actual_w = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.capture.get(cv2.CAP_PROP_FPS)
        if (actual_w, actual_h) != (width, height):
            self.get_logger().warning(
                f'Requested {width}x{height} but device reports '
                f'{actual_w}x{actual_h}. Check `v4l2-ctl --list-formats-ext '
                f'-d {device}` for supported modes.'
            )

        # Same QoS profile camera_navigator subscribes with, so the
        # publisher/subscriber pair is compatible and low-latency.
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Image, '/camera/image', sensor_qos)

        self.timer = self.create_timer(1.0 / fps, self._capture_and_publish)
        self.get_logger().info(
            f'camera_node started on {device} ({width}x{height} @ {fps} fps), '
            'publishing /camera/image'
        )

    def _capture_and_publish(self):
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.get_logger().warning(
                'Failed to read frame from camera.', throttle_duration_sec=5.0
            )
            return

        if self.flip_h:
            frame = cv2.flip(frame, 1)
        if self.flip_v:
            frame = cv2.flip(frame, 0)

        message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)

    def destroy_node(self) -> bool:
        if self.capture is not None:
            self.capture.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()