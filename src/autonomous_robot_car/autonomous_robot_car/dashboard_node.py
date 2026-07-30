"""Dependency-free HTTP dashboard backed by ROS 2 topics."""

from functools import partial
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
from queue import Empty, Queue
import threading
import time

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


ALLOWED_COMMANDS = {
    'start:red', 'start:green', 'start:blue', 'start:patrol',
    'pause', 'resume', 'stop', 'cancel', 'delivered',
}


class DashboardHandler(SimpleHTTPRequestHandler):
    """Serve static UI assets and a tiny JSON API."""

    server_version = 'RobotDashboard/0.1'

    def log_message(self, fmt, *args):
        # HTTP access logs obscure the useful ROS status output.
        return

    @property
    def dashboard(self):
        return self.server.dashboard_node

    def _json_response(self, payload, status=HTTPStatus.OK):
        encoded = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(encoded)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(encoded)

    def do_GET(self):
        if self.path.startswith('/api/status'):
            self._json_response(self.dashboard.status_snapshot())
            return
        if self.path.startswith('/camera.jpg'):
            image = self.dashboard.camera_snapshot()
            if image is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, 'Camera frame unavailable')
                return
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(image)))
            self.send_header('Cache-Control', 'no-store, max-age=0')
            self.end_headers()
            self.wfile.write(image)
            return
        if self.path == '/':
            self.path = '/index.html'
        super().do_GET()

    def do_POST(self):
        if self.path != '/api/command':
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            size = int(self.headers.get('Content-Length', '0'))
            payload = json.loads(self.rfile.read(min(size, 2048)) or b'{}')
            command = str(payload.get('command', '')).strip().lower()
        except (ValueError, json.JSONDecodeError):
            self._json_response({'ok': False, 'error': 'Invalid JSON'}, HTTPStatus.BAD_REQUEST)
            return
        if command in ('payload:loaded', 'payload:empty'):
            self.dashboard.set_payload(command.endswith('loaded'))
        elif command in ALLOWED_COMMANDS:
            self.dashboard.queue_command(command)
        else:
            self._json_response(
                {'ok': False, 'error': 'Unsupported command'},
                HTTPStatus.BAD_REQUEST,
            )
            return
        self._json_response({'ok': True, 'command': command})


class DashboardNode(Node):
    """Collect robot telemetry and host the browser dashboard."""

    def __init__(self) -> None:
        super().__init__('delivery_dashboard')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('jpeg_quality', 78)
        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.bridge = CvBridge()
        self.command_publisher = self.create_publisher(String, '/mission/command', 10)
        self.create_subscription(Image, '/camera/image', self._image_callback, sensor_qos)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(String, '/autonomy/status', self._status_callback, state_qos)

        self.lock = threading.Lock()
        self.commands = Queue()
        self.latest_jpeg = None
        self.last_frame_wall_time = 0.0
        self.payload_loaded = True
        self.nav_status = {
            'state': 'BOOTING',
            'destination': 'NONE',
            'detail': 'Waiting for navigation status.',
            'enabled': False,
            'arrived': False,
            'camera_ok': False,
            'front_distance': None,
            'target_visible': False,
            'commanded_linear': 0.0,
            'commanded_angular': 0.0,
        }
        self.odom_status = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'speed': 0.0}
        self.started_wall_time = time.time()
        self.create_timer(0.05, self._drain_commands)

        asset_directory = Path(get_package_share_directory('autonomous_robot_car')) / 'dashboard'
        handler = partial(DashboardHandler, directory=str(asset_directory))
        self.http_server = ThreadingHTTPServer((self.host, self.port), handler)
        self.http_server.dashboard_node = self
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            name='delivery-dashboard-http',
            daemon=True,
        )
        self.http_thread.start()
        display_host = 'localhost' if self.host in ('0.0.0.0', '::') else self.host
        self.get_logger().info(f'Delivery dashboard: http://{display_host}:{self.port}')

    def _image_callback(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            ok, jpeg = cv2.imencode(
                '.jpg', frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if ok:
                with self.lock:
                    self.latest_jpeg = jpeg.tobytes()
                    self.last_frame_wall_time = time.time()
        except CvBridgeError as error:
            self.get_logger().warning(
                f'Dashboard image conversion failed: {error}',
                throttle_duration_sec=5.0,
            )

    def _odom_callback(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        linear = message.twist.twist.linear
        with self.lock:
            self.odom_status = {
                'x': round(message.pose.pose.position.x, 2),
                'y': round(message.pose.pose.position.y, 2),
                'yaw': round(math.degrees(yaw), 1),
                'speed': round(math.hypot(linear.x, linear.y), 3),
            }

    def _status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            if isinstance(status, dict):
                with self.lock:
                    self.nav_status = status
        except json.JSONDecodeError:
            self.get_logger().warning(
                'Received malformed /autonomy/status JSON',
                throttle_duration_sec=5.0,
            )

    def queue_command(self, command: str) -> None:
        self.commands.put(command)

    def set_payload(self, loaded: bool) -> None:
        with self.lock:
            self.payload_loaded = loaded

    def _drain_commands(self) -> None:
        while True:
            try:
                command = self.commands.get_nowait()
            except Empty:
                break
            message = String()
            message.data = command
            self.command_publisher.publish(message)
            self.get_logger().info(f'Dashboard command: {command}')

    def camera_snapshot(self):
        with self.lock:
            return self.latest_jpeg

    def status_snapshot(self):
        with self.lock:
            data = dict(self.nav_status)
            data['odometry'] = dict(self.odom_status)
            data['payload_loaded'] = self.payload_loaded
            data['payload_grams'] = 100 if self.payload_loaded else 0
            data['camera_stream_age'] = (
                round(time.time() - self.last_frame_wall_time, 2)
                if self.last_frame_wall_time else None
            )
        data['dashboard_uptime'] = round(time.time() - self.started_wall_time)
        return data

    def destroy_node(self) -> bool:
        self.http_server.shutdown()
        self.http_server.server_close()
        self.http_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
