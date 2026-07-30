"""Reactive RGB-D navigation and color-station delivery controller."""

import json
import math
from typing import Optional

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .navigation_logic import (
    DepthSectors,
    find_color_target,
    LocalPathPlan,
    measure_depth_sectors,
    normalize_depth,
    panoramic_depth_profile,
    select_exploration_heading,
    select_local_path,
    TargetObservation,
)


class CameraNavigator(Node):
    """Navigate without a laser scanner, using RGB target cues and camera depth."""

    def __init__(self) -> None:
        super().__init__('camera_navigator')
        self._declare_parameters()
        self.bridge = CvBridge()

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
        self.create_subscription(Image, '/camera/image', self._rgb_callback, sensor_qos)
        self.create_subscription(Image, '/camera/depth_image', self._depth_callback, sensor_qos)
        self.create_subscription(
            Image,
            '/camera/left/depth_image',
            lambda message: self._surround_depth_callback('left', message),
            sensor_qos,
        )
        self.create_subscription(
            Image,
            '/camera/right/depth_image',
            lambda message: self._surround_depth_callback('right', message),
            sensor_qos,
        )
        self.create_subscription(
            Image,
            '/camera/rear/depth_image',
            lambda message: self._surround_depth_callback('rear', message),
            sensor_qos,
        )
        self.create_subscription(
            Image,
            '/camera/left/image',
            lambda message: self._surround_rgb_callback('left', message),
            sensor_qos,
        )
        self.create_subscription(
            Image,
            '/camera/right/image',
            lambda message: self._surround_rgb_callback('right', message),
            sensor_qos,
        )
        self.create_subscription(
            Image,
            '/camera/rear/image',
            lambda message: self._surround_rgb_callback('rear', message),
            sensor_qos,
        )
        self.create_subscription(String, '/mission/command', self._command_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odometry_callback, 10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, '/autonomy/status', state_qos)

        self.rgb_image: Optional[np.ndarray] = None
        self.depth_image: Optional[np.ndarray] = None
        self.last_rgb_time = -math.inf
        self.last_depth_time = -math.inf
        self.surround_depth = {'left': None, 'right': None, 'rear': None}
        self.surround_rgb = {'left': None, 'right': None, 'rear': None}
        self.last_surround_time = {
            'left': -math.inf,
            'right': -math.inf,
            'rear': -math.inf,
        }
        self.enabled = False
        self.destination = 'none'
        self.state = 'IDLE'
        self.detail = 'Select a destination and start a mission.'
        self.arrived = False
        self.search_direction = 1.0
        self.alignment_direction = 0.0
        self.exploration_active = False
        self.exploration_world_heading: Optional[float] = None
        self.exploration_start_x = 0.0
        self.exploration_start_y = 0.0
        self.exploration_best_distance = 0.0
        self.exploration_last_progress = 0.0
        self.failed_exploration_world_heading: Optional[float] = None
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.sectors = DepthSectors(self.maximum_depth, self.maximum_depth, self.maximum_depth)
        self.left_clearance = self.maximum_depth
        self.right_clearance = self.maximum_depth
        self.rear_clearance = self.maximum_depth
        self.target: Optional[TargetObservation] = None
        self.surround_target: Optional[tuple[str, TargetObservation]] = None
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.goal_source: Optional[str] = None
        self.path_plan: Optional[LocalPathPlan] = None
        self.planned_world_heading: Optional[float] = None
        self.profile_angles = np.empty(0, dtype=np.float32)
        self.profile_ranges = np.empty(0, dtype=np.float32)

        rate = max(2.0, float(self.get_parameter('command_rate').value))
        self.timer = self.create_timer(1.0 / rate, self._control_tick)
        self.get_logger().info(
            'Camera-only navigator ready. Send start:red, start:green, '
            'start:blue, or start:patrol on /mission/command.'
        )

    def _declare_parameters(self) -> None:
        defaults = {
            'cruise_speed': 0.22,
            'search_speed': 0.13,
            'approach_speed': 0.17,
            'cautious_speed': 0.08,
            'emergency_distance': 0.42,
            'stop_distance': 0.55,
            'turn_clearance': 0.27,
            'slow_distance': 1.10,
            'arrival_distance': 0.72,
            'target_min_area_fraction': 0.001,
            'camera_timeout': 1.0,
            'command_rate': 10.0,
            'maximum_depth': 8.0,
            'horizontal_fov': 1.74533,
            'surround_horizontal_fov': 1.74533,
            'planner_bins': 72,
            'planner_candidates': 33,
            'planner_max_heading': 1.39626,
            'planner_heading_gain': 0.95,
            'planner_hysteresis': 1.25,
            'planner_switch_penalty': 0.75,
            'robot_width': 0.43,
            'gap_safety_margin': 0.05,
            'use_known_bay_poses': True,
            # Fixed simulation bay approach poses in the startup odom frame.
            # On hardware these values should come from the saved visual map.
            'red_bay_x': 3.85,
            'red_bay_y': -2.48,
            'green_bay_x': 3.85,
            'green_bay_y': 2.48,
            'blue_bay_x': 4.68,
            'blue_bay_y': -0.95,
            'known_goal_search_radius': 0.90,
            'exploration_distance': 1.00,
            'exploration_stall_timeout': 9.0,
            'exploration_clearance': 0.75,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, value)
        for name in defaults:
            setattr(self, name, self.get_parameter(name).value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _rgb_callback(self, message: Image) -> None:
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            self.last_rgb_time = self._now()
        except CvBridgeError as error:
            self.get_logger().warning(
                f'Could not decode RGB image: {error}',
                throttle_duration_sec=5.0,
            )

    def _depth_callback(self, message: Image) -> None:
        try:
            raw = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
            self.depth_image = normalize_depth(raw, message.encoding)
            self.last_depth_time = self._now()
        except CvBridgeError as error:
            self.get_logger().warning(
                f'Could not decode depth image: {error}',
                throttle_duration_sec=5.0,
            )

    def _surround_depth_callback(self, direction: str, message: Image) -> None:
        try:
            raw = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
            self.surround_depth[direction] = normalize_depth(raw, message.encoding)
            self.last_surround_time[direction] = self._now()
        except CvBridgeError as error:
            self.get_logger().warning(
                f'Could not decode {direction} depth image: {error}',
                throttle_duration_sec=5.0,
            )

    def _surround_rgb_callback(self, direction: str, message: Image) -> None:
        try:
            self.surround_rgb[direction] = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )
        except CvBridgeError as error:
            self.get_logger().warning(
                f'Could not decode {direction} RGB image: {error}',
                throttle_duration_sec=5.0,
            )

    def _odometry_callback(self, message: Odometry) -> None:
        self.odom_x = float(message.pose.pose.position.x)
        self.odom_y = float(message.pose.pose.position.y)
        orientation = message.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        self.odom_yaw = math.atan2(sin_yaw, cos_yaw)

    def _command_callback(self, message: String) -> None:
        command = message.data.strip().lower()
        if command.startswith('start:'):
            destination = command.partition(':')[2]
            if destination not in ('red', 'green', 'blue', 'patrol'):
                self.get_logger().warning(f'Ignoring unknown destination: {destination}')
                return
            self.destination = destination
            self.enabled = True
            self.arrived = False
            self.search_direction = (
                1.0 if self.left_clearance >= self.right_clearance else -1.0
            )
            self.alignment_direction = 0.0
            self._reset_exploration()
            self._set_known_bay_goal(destination)
            self.path_plan = None
            self.planned_world_heading = None
            self.state = 'STARTING'
            self.detail = f'Starting {destination} mission.'
            self.get_logger().info(self.detail)
        elif command == 'pause':
            self.enabled = False
            self.state = 'PAUSED'
            self.detail = 'Mission paused by operator.'
            self._publish_velocity(0.0, 0.0)
        elif command == 'resume' and self.destination != 'none' and not self.arrived:
            self.enabled = True
            self.state = 'RESUMING'
            self.detail = 'Mission resumed.'
        elif command in ('stop', 'cancel'):
            self.enabled = False
            self.destination = 'none'
            self.arrived = False
            self.state = 'IDLE'
            self.detail = 'Mission stopped.'
            self.goal_x = None
            self.goal_y = None
            self.goal_source = None
            self._reset_exploration()
            self.path_plan = None
            self.planned_world_heading = None
            self._publish_velocity(0.0, 0.0)
        elif command == 'delivered' and self.arrived:
            self.enabled = False
            self.destination = 'none'
            self.arrived = False
            self.state = 'DELIVERED'
            self.detail = 'Delivery acknowledged. Robot is ready.'

    def _camera_is_fresh(self, now: float) -> bool:
        depth_fresh = now - self.last_depth_time <= self.camera_timeout
        surround_fresh = all(
            now - timestamp <= self.camera_timeout
            for timestamp in self.last_surround_time.values()
        )
        if self.destination in ('red', 'green', 'blue'):
            return (
                depth_fresh and surround_fresh and
                now - self.last_rgb_time <= self.camera_timeout
            )
        return depth_fresh and surround_fresh

    def _remember_target(self, observation: TargetObservation, bearing: float) -> bool:
        """Store a camera-derived goal point for short visual occlusions."""
        if not 0.08 < observation.distance < self.maximum_depth:
            goal_error = self._goal_bearing_error()
            if goal_error is None:
                return True
            disagreement = math.atan2(
                math.sin(bearing - goal_error),
                math.cos(bearing - goal_error),
            )
            return abs(disagreement) < 0.75
        world_bearing = self.odom_yaw + bearing
        observed_x = self.odom_x + observation.distance * math.cos(world_bearing)
        observed_y = self.odom_y + observation.distance * math.sin(world_bearing)
        if self.goal_x is not None and self.goal_y is not None:
            disagreement = math.hypot(
                observed_x - self.goal_x,
                observed_y - self.goal_y,
            )
            if disagreement > 1.2:
                goal_error = self._goal_bearing_error()
                bearing_disagreement = math.atan2(
                    math.sin(bearing - goal_error),
                    math.cos(bearing - goal_error),
                )
                # Preserve the established range estimate, but a color cue
                # pointing the same way is still useful for direct centring.
                return abs(bearing_disagreement) < 0.75
            # Smooth RGB-D edge noise so it cannot steer the remembered goal
            # abruptly when a delivery panel is partially occluded.
            observed_x = 0.75 * self.goal_x + 0.25 * observed_x
            observed_y = 0.75 * self.goal_y + 0.25 * observed_y
        self.goal_x = observed_x
        self.goal_y = observed_y
        self.goal_source = 'visual'
        return True

    def _set_known_bay_goal(self, destination: str) -> None:
        """Load a mapped bay approach pose so visibility is not required to start."""
        self.goal_x = None
        self.goal_y = None
        self.goal_source = None
        if not self.use_known_bay_poses or destination == 'patrol':
            return
        self.goal_x = float(getattr(self, f'{destination}_bay_x'))
        self.goal_y = float(getattr(self, f'{destination}_bay_y'))
        self.goal_source = 'known'

    def _goal_distance(self) -> Optional[float]:
        if self.goal_x is None or self.goal_y is None:
            return None
        return math.hypot(self.goal_x - self.odom_x, self.goal_y - self.odom_y)

    def _reset_exploration(self) -> None:
        self.exploration_active = False
        self.exploration_world_heading = None
        self.exploration_best_distance = 0.0
        self.exploration_last_progress = self._now()

    def _start_exploration(self, now: float) -> None:
        """Select and lock a camera-confirmed route to a genuinely new viewpoint."""
        failed_heading = None
        if self.failed_exploration_world_heading is not None:
            failed_heading = math.atan2(
                math.sin(self.failed_exploration_world_heading - self.odom_yaw),
                math.cos(self.failed_exploration_world_heading - self.odom_yaw),
            )
        route = select_exploration_heading(
            self.profile_angles,
            self.profile_ranges,
            failed_heading,
            float(self.robot_width),
            float(self.gap_safety_margin),
            float(self.exploration_clearance),
            float(self.maximum_depth),
        )
        self.path_plan = route
        self.exploration_world_heading = self.odom_yaw + route.heading
        self.exploration_start_x = self.odom_x
        self.exploration_start_y = self.odom_y
        self.exploration_best_distance = 0.0
        self.exploration_last_progress = now
        self.exploration_active = route.passable

    def _explore(self, now: float) -> tuple[float, float]:
        if not self.exploration_active or self.exploration_world_heading is None:
            self._start_exploration(now)
        if not self.exploration_active or self.exploration_world_heading is None:
            self.state = 'EXPLORATION_BLOCKED'
            self.detail = 'No camera-confirmed route is wide enough for exploration.'
            return 0.0, 0.0

        distance = math.hypot(
            self.odom_x - self.exploration_start_x,
            self.odom_y - self.exploration_start_y,
        )
        if distance >= self.exploration_best_distance + 0.05:
            self.exploration_best_distance = distance
            self.exploration_last_progress = now

        if distance >= self.exploration_distance:
            self.failed_exploration_world_heading = None
            self._reset_exploration()
            self._start_exploration(now)
        elif now - self.exploration_last_progress >= self.exploration_stall_timeout:
            self.failed_exploration_world_heading = self.exploration_world_heading
            self._reset_exploration()
            self._start_exploration(now)

        if not self.exploration_active or self.exploration_world_heading is None:
            self.state = 'EXPLORATION_BLOCKED'
            self.detail = 'All alternate camera routes are blocked.'
            return 0.0, 0.0
        heading_error = math.atan2(
            math.sin(self.exploration_world_heading - self.odom_yaw),
            math.cos(self.exploration_world_heading - self.odom_yaw),
        )
        self.state = 'EXPLORING'
        self.detail = (
            'Bay is not visible; moving to a new camera viewpoint '
            f'through a {self.path_plan.gap_width:.2f} m gap.'
        )
        return float(self.search_speed), float(heading_error)

    def _goal_bearing_error(self) -> Optional[float]:
        if self.goal_x is None or self.goal_y is None:
            return None
        world_bearing = math.atan2(
            self.goal_y - self.odom_y,
            self.goal_x - self.odom_x,
        )
        return math.atan2(
            math.sin(world_bearing - self.odom_yaw),
            math.cos(world_bearing - self.odom_yaw),
        )

    def _previous_path_heading(self) -> float:
        if self.planned_world_heading is None:
            return 0.0
        return math.atan2(
            math.sin(self.planned_world_heading - self.odom_yaw),
            math.cos(self.planned_world_heading - self.odom_yaw),
        )

    def _follow_local_path(
        self,
        requested_speed: float,
        desired_heading: float,
        minimum_clearance: float,
    ) -> tuple[float, float]:
        """Follow a footprint-safe polar gap without left/right oscillation."""
        previous_heading = self._previous_path_heading()
        self.path_plan = select_local_path(
            self.profile_angles,
            self.profile_ranges,
            desired_heading,
            previous_heading,
            float(self.robot_width),
            float(self.gap_safety_margin),
            minimum_clearance,
            float(self.maximum_depth),
            float(self.planner_max_heading),
            int(self.planner_candidates),
            float(self.planner_hysteresis),
            float(self.planner_switch_penalty),
        )
        heading = self.path_plan.heading
        self.planned_world_heading = self.odom_yaw + heading
        angular = float(np.clip(
            self.planner_heading_gain * heading,
            -0.72,
            0.72,
        ))

        if not self.path_plan.passable:
            self.state = 'PATH_BLOCKED'
            self.detail = 'No footprint-safe camera gap; rotating toward the best opening.'
            return 0.0, angular

        clearance_scale = np.clip(
            (self.path_plan.clearance - minimum_clearance) /
            max(self.slow_distance - minimum_clearance, 0.1),
            0.0,
            1.0,
        )
        turn_scale = max(0.25, math.cos(heading) ** 2)
        linear = requested_speed * (0.30 + 0.70 * clearance_scale) * turn_scale
        if abs(heading) > 0.58:
            linear = 0.0

        if abs(heading - desired_heading) > 0.10:
            self.state = 'PATH_FOLLOWING'
            self.detail = (
                f'Following a {self.path_plan.gap_width:.2f} m camera-confirmed gap.'
            )
        return float(linear), angular

    def _search_or_patrol(self, now: float) -> tuple[float, float]:
        if self.destination == 'patrol':
            self.state = 'PATROLLING'
            self.detail = 'Roaming with camera-only obstacle avoidance.'
            steer = 0.22 * np.clip(
                (self.sectors.left - self.sectors.right) / max(self.maximum_depth, 0.1),
                -1.0,
                1.0,
            )
            return float(self.cruise_speed), float(steer + 0.05 * self.search_direction)
        goal_error = self._goal_bearing_error()
        if goal_error is not None:
            goal_distance = self._goal_distance()
            if (
                self.goal_source == 'known' and
                goal_distance is not None and
                goal_distance <= self.known_goal_search_radius
            ):
                # The mapped pose is an approach point, not proof of delivery.
                # Search from nearby viewpoints until vision confirms the bay.
                self.goal_x = None
                self.goal_y = None
                self.goal_source = None
                self._reset_exploration()
                return self._explore(now)
            self._reset_exploration()
            if self.goal_source == 'known':
                self.state = 'ROUTING_TO_BAY'
                self.detail = (
                    f'{self.destination.upper()} bay is hidden; following its saved pose.'
                )
            else:
                self.state = 'VISUAL_MEMORY'
                self.detail = 'Bay is occluded; advancing on its last RGB-D observation.'
            return float(self.search_speed), float(goal_error)
        return self._explore(now)

    def _control_tick(self) -> None:
        now = self._now()
        linear = 0.0
        angular = 0.0
        self.target = None
        self.surround_target = None

        if self.depth_image is not None:
            self.sectors = measure_depth_sectors(
                self.depth_image,
                self.maximum_depth,
                self.horizontal_fov,
            )
        for direction, depth_image in self.surround_depth.items():
            if depth_image is None:
                continue
            surround_sectors = measure_depth_sectors(
                depth_image,
                self.maximum_depth,
                self.surround_horizontal_fov,
            )
            # The centre of each pod is the true left/right/rear clearance.
            # Taking the minimum over its full 100-degree image lets a front
            # obstacle leak into a side reading and can make the robot reverse
            # its turn forever even when the side itself is open.
            clearance = surround_sectors.center
            setattr(self, f'{direction}_clearance', clearance)

        depth_views = []
        if self.depth_image is not None:
            depth_views.append((self.depth_image, float(self.horizontal_fov), 0.0))
        view_yaws = {
            'left': math.pi / 2.0,
            'right': -math.pi / 2.0,
            'rear': math.pi,
        }
        for direction, depth_image in self.surround_depth.items():
            if depth_image is not None:
                depth_views.append((
                    depth_image,
                    float(self.surround_horizontal_fov),
                    view_yaws[direction],
                ))
        self.profile_angles, self.profile_ranges = panoramic_depth_profile(
            depth_views,
            float(self.maximum_depth),
            int(self.planner_bins),
        )

        if not self.enabled:
            if self.arrived:
                self.state = 'ARRIVED'
                self.detail = f'Waiting at the {self.destination.upper()} delivery bay.'
        elif not self._camera_is_fresh(now):
            self.state = 'CAMERA_LOST'
            self.detail = 'Safety stop: camera stream is missing or stale.'
        elif self.arrived:
            self.enabled = False
            self.state = 'ARRIVED'
        else:
            if self.destination in ('red', 'green', 'blue') and self.rgb_image is not None:
                self.target = find_color_target(
                    self.rgb_image,
                    self.depth_image,
                    self.destination,
                    float(self.target_min_area_fraction),
                    float(self.maximum_depth),
                    float(self.horizontal_fov),
                )
                if self.target is not None:
                    self.alignment_direction = 0.0
                    self._reset_exploration()
                    front_bearing = (
                        -self.target.horizontal_error * self.horizontal_fov / 2.0
                    )
                    if not self._remember_target(self.target, front_bearing):
                        self.target = None
                else:
                    candidates = []
                    for direction, image in self.surround_rgb.items():
                        depth_image = self.surround_depth[direction]
                        if image is None or depth_image is None:
                            continue
                        observation = find_color_target(
                            image,
                            depth_image,
                            self.destination,
                            float(self.target_min_area_fraction),
                            float(self.maximum_depth),
                            float(self.surround_horizontal_fov),
                        )
                        if observation is not None:
                            candidates.append((direction, observation))
                    if candidates:
                        self.surround_target = max(
                            candidates,
                            key=lambda item: item[1].area_fraction,
                        )
                        self._reset_exploration()
                        direction, observation = self.surround_target
                        camera_bearing = {
                            'left': math.pi / 2.0,
                            'right': -math.pi / 2.0,
                            'rear': math.pi,
                        }[direction]
                        bearing = (
                            camera_bearing -
                            observation.horizontal_error *
                            self.surround_horizontal_fov / 2.0
                        )
                        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
                        if not self._remember_target(observation, bearing):
                            # A tiny marker sliver can contain background depth.
                            # Do not let that intermittent outlier fight the
                            # accepted visual-memory heading every other tick.
                            self.surround_target = None

            target_is_centered = (
                self.target is not None and
                abs(self.target.horizontal_error) <= 0.25
            )
            if self.target is not None:
                if self.target.distance <= self.arrival_distance:
                    self.arrived = True
                    self.enabled = False
                    self.state = 'ARRIVED'
                    self.detail = f'Arrived at the {self.destination.upper()} delivery bay.'
                elif not target_is_centered:
                    self.state = 'CENTERING_TARGET'
                    self.detail = f'Centering the {self.destination.upper()} bay in front RGB.'
                    linear = float(self.search_speed * 0.50)
                    angular = float(
                        -self.target.horizontal_error * self.horizontal_fov / 2.0
                    )
                    if abs(angular) > 0.10:
                        self.search_direction = 1.0 if angular > 0.0 else -1.0
                else:
                    self.state = 'APPROACHING'
                    self.detail = f'{self.destination.upper()} bay acquired; approaching visually.'
                    angular = float(
                        -self.target.horizontal_error * self.horizontal_fov / 2.0
                    )
                    heading_factor = max(0.30, 1.0 - abs(self.target.horizontal_error))
                    linear = float(self.approach_speed * heading_factor)
                    if abs(angular) > 0.10:
                        self.search_direction = 1.0 if angular > 0.0 else -1.0
            elif self.surround_target is not None:
                direction, observation = self.surround_target
                self.state = 'ALIGNING_TARGET'
                self.detail = f'Aligning front camera from the {direction} pod view.'
                linear = 0.0
                camera_bearing = {
                    'left': math.pi / 2.0,
                    'right': -math.pi / 2.0,
                    'rear': math.pi,
                }[direction]
                target_bearing = (
                    camera_bearing -
                    observation.horizontal_error * self.surround_horizontal_fov / 2.0
                )
                target_bearing = math.atan2(
                    math.sin(target_bearing),
                    math.cos(target_bearing),
                )
                linear = float(self.search_speed * 0.50)
                angular = float(target_bearing)
                self.alignment_direction = 1.0 if angular > 0.0 else -1.0
                self.search_direction = self.alignment_direction
            else:
                linear, angular = self._search_or_patrol(now)

            if linear > 0.0:
                minimum_clearance = float(self.stop_distance)
                if (
                    self.target is not None and
                    self.target.distance < self.stop_distance + 0.15
                ):
                    minimum_clearance = max(
                        float(self.emergency_distance),
                        float(self.arrival_distance) * 0.80,
                    )
                desired_heading = float(np.clip(
                    angular,
                    -self.planner_max_heading,
                    self.planner_max_heading,
                ))
                linear, angular = self._follow_local_path(
                    linear,
                    desired_heading,
                    minimum_clearance,
                )

        # Pure in-place turns still need a swept-corner guard. Translational
        # turns are already footprint-inflated by the polar gap planner.
        if self.enabled and linear <= 0.001 and abs(angular) > 0.10:
            turning_clearance = (
                self.left_clearance if angular > 0.0 else self.right_clearance
            )
            if turning_clearance < self.turn_clearance:
                angular = 0.0
                self.state = 'TRAPPED'
                self.detail = 'Safety stop: insufficient swept-corner clearance.'

        self._publish_velocity(linear, angular)
        self._publish_status(now)

    def _publish_velocity(self, linear: float, angular: float) -> None:
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self.cmd_publisher.publish(command)
        self.last_linear = float(linear)
        self.last_angular = float(angular)

    def _publish_status(self, now: float) -> None:
        camera_times = [self.last_depth_time, *self.last_surround_time.values()]
        camera_age = (
            max(now - timestamp for timestamp in camera_times)
            if all(math.isfinite(timestamp) for timestamp in camera_times)
            else None
        )
        status = {
            'state': self.state,
            'destination': self.destination.upper(),
            'detail': self.detail,
            'enabled': self.enabled,
            'arrived': self.arrived,
            'camera_ok': self._camera_is_fresh(now),
            'camera_age': round(camera_age, 2) if camera_age is not None else None,
            'front_distance': round(self.sectors.center, 2),
            'front_left_distance': round(self.sectors.left, 2),
            'front_right_distance': round(self.sectors.right, 2),
            'left_distance': round(self.left_clearance, 2),
            'right_distance': round(self.right_clearance, 2),
            'rear_distance': round(self.rear_clearance, 2),
            'target_visible': self.target is not None or self.surround_target is not None,
            'target_distance': (
                round(self.target.distance, 2)
                if self.target else
                round(self.surround_target[1].distance, 2)
                if self.surround_target else None
            ),
            'planned_heading_degrees': (
                round(math.degrees(self.path_plan.heading), 1)
                if self.path_plan and self.enabled else None
            ),
            'planned_clearance': (
                round(self.path_plan.clearance, 2)
                if self.path_plan and self.enabled else None
            ),
            'planned_gap_width': (
                round(self.path_plan.gap_width, 2)
                if self.path_plan and self.enabled else None
            ),
            'commanded_linear': round(self.last_linear, 3),
            'commanded_angular': round(self.last_angular, 3),
        }
        message = String()
        message.data = json.dumps(status, separators=(',', ':'))
        self.status_publisher.publish(message)

    def destroy_node(self) -> bool:
        if rclpy.ok():
            self._publish_velocity(0.0, 0.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNavigator()
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
