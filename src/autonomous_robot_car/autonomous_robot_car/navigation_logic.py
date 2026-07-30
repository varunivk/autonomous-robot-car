"""Image-processing helpers kept independent from ROS for unit testing."""

from dataclasses import dataclass
import math
from typing import Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class DepthSectors:
    """Robust near-distance estimates for the lower camera field of view."""

    left: float
    center: float
    right: float


@dataclass(frozen=True)
class TargetObservation:
    """Location and camera-depth estimate for a color delivery target."""

    horizontal_error: float
    area_fraction: float
    distance: float


@dataclass(frozen=True)
class LocalPathPlan:
    """Selected collision-free heading from the panoramic depth profile."""

    heading: float
    clearance: float
    gap_width: float
    passable: bool


HSV_RANGES = {
    'red': (
        (np.array([0, 100, 80]), np.array([10, 255, 255])),
        (np.array([170, 100, 80]), np.array([180, 255, 255])),
    ),
    'green': (
        (np.array([38, 75, 55]), np.array([88, 255, 255])),
    ),
    'blue': (
        # Delivery panels are deliberately vivid. The higher saturation floor
        # rejects ordinary blue-grey furniture such as the test-room ottoman.
        (np.array([92, 165, 70]), np.array([132, 255, 255])),
    ),
}


def normalize_depth(depth: np.ndarray, encoding: str = '') -> np.ndarray:
    """Convert a ROS depth image to metres and replace invalid values with NaN."""
    values = np.asarray(depth, dtype=np.float32)
    if encoding.upper() in ('16UC1', 'MONO16'):
        values = values / 1000.0
    values[(~np.isfinite(values)) | (values <= 0.08)] = np.nan
    return values


def _near_percentile(region: np.ndarray, maximum_depth: float) -> float:
    valid = region[np.isfinite(region) & (region <= maximum_depth)]
    if valid.size < 12:
        return maximum_depth
    # A low robust percentile sees narrow furniture legs without trusting a
    # single potentially noisy depth pixel.
    return float(np.percentile(valid, 2.0))


def horizontal_ray_ranges(depth: np.ndarray, horizontal_fov: float) -> np.ndarray:
    """Convert optical-axis camera depth to horizontal radial range."""
    if depth.ndim != 2 or depth.size == 0:
        return depth
    width = depth.shape[1]
    normalized_x = (2.0 * (np.arange(width, dtype=np.float32) + 0.5) / width) - 1.0
    ray_x = normalized_x * math.tan(horizontal_fov / 2.0)
    range_factor = np.sqrt(1.0 + np.square(ray_x))
    return depth * range_factor[np.newaxis, :]


def measure_depth_sectors(
    depth: np.ndarray,
    maximum_depth: float = 8.0,
    horizontal_fov: float = 1.22173,
) -> DepthSectors:
    """Measure three overlapping navigation sectors in the useful lower image band."""
    if depth.ndim != 2 or depth.size == 0:
        return DepthSectors(maximum_depth, maximum_depth, maximum_depth)
    depth_range = horizontal_ray_ranges(depth, horizontal_fov)
    height, width = depth_range.shape
    # The 150-degree lens puts a large ground plane in the lower image. Sample
    # a narrow band around camera height: room obstacles intersect this band,
    # while ordinary floor pixels cannot become false emergency obstacles.
    y0, y1 = int(height * 0.36), int(height * 0.42)
    band = depth_range[y0:y1, :]
    return DepthSectors(
        _near_percentile(band[:, :int(width * 0.40)], maximum_depth),
        _near_percentile(band[:, int(width * 0.30):int(width * 0.70)], maximum_depth),
        _near_percentile(band[:, int(width * 0.60):], maximum_depth),
    )


def panoramic_depth_profile(
    views: list[tuple[np.ndarray, float, float]],
    maximum_depth: float = 8.0,
    bin_count: int = 72,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Fuse overlapping camera views into a robust 360-degree polar profile.

    Each view tuple contains ``(depth, horizontal_fov, camera_yaw)``. Positive
    yaw is left. The returned angles are bin centres in ``[-pi, pi)``.
    """
    bin_count = max(16, int(bin_count))
    bin_width = 2.0 * math.pi / bin_count
    angles = -math.pi + (np.arange(bin_count, dtype=np.float32) + 0.5) * bin_width
    buckets: list[list[float]] = [[] for _ in range(bin_count)]

    for depth, horizontal_fov, camera_yaw in views:
        if depth is None or depth.ndim != 2 or depth.size == 0:
            continue
        radial = horizontal_ray_ranges(depth, horizontal_fov)
        height, width = radial.shape
        y0, y1 = int(height * 0.34), max(int(height * 0.44), int(height * 0.34) + 1)
        band = radial[y0:y1, :]
        normalized_x = (
            2.0 * (np.arange(width, dtype=np.float32) + 0.5) / width
        ) - 1.0
        ray_angles = camera_yaw - np.arctan(
            normalized_x * math.tan(horizontal_fov / 2.0)
        )
        ray_angles = (ray_angles + math.pi) % (2.0 * math.pi) - math.pi

        for column in range(width):
            valid = band[:, column]
            valid = valid[
                np.isfinite(valid) & (valid > 0.08) & (valid <= maximum_depth)
            ]
            if valid.size < 2:
                continue
            distance = float(np.percentile(valid, 10.0))
            index = int((ray_angles[column] + math.pi) / bin_width) % bin_count
            buckets[index].append(distance)

    ranges = np.full(bin_count, maximum_depth, dtype=np.float32)
    for index, values in enumerate(buckets):
        if values:
            # Overlapping pod views may hit an edge differently. A low robust
            # percentile retains poles while rejecting isolated depth noise.
            ranges[index] = float(np.percentile(values, 10.0))
    return angles, ranges


def select_local_path(
    profile_angles: np.ndarray,
    profile_ranges: np.ndarray,
    desired_heading: float,
    previous_heading: float = 0.0,
    robot_width: float = 0.43,
    safety_margin: float = 0.05,
    minimum_clearance: float = 0.62,
    maximum_depth: float = 8.0,
    maximum_heading: float = 1.39626,
    candidate_count: int = 33,
    hysteresis_weight: float = 1.25,
    switch_penalty: float = 0.75,
) -> LocalPathPlan:
    """
    Choose a VFH-style forward heading with footprint-inflated clearance.

    The robot is treated as a corridor of its physical width plus a margin.
    This tests actual gap width instead of reacting to one left/right range.
    Hysteresis keeps the selected side stable at obstacle edges.
    """
    if (
        profile_angles.size == 0 or
        profile_ranges.size != profile_angles.size
    ):
        return LocalPathPlan(0.0, 0.0, 0.0, False)

    candidate_count = max(9, int(candidate_count) | 1)
    candidates = np.linspace(
        -maximum_heading,
        maximum_heading,
        candidate_count,
        dtype=np.float32,
    )
    desired = float(np.clip(desired_heading, -maximum_heading, maximum_heading))
    previous = float(np.clip(previous_heading, -maximum_heading, maximum_heading))
    corridor_half_width = robot_width / 2.0 + safety_margin

    observed = np.isfinite(profile_ranges) & (profile_ranges < maximum_depth * 0.995)
    obstacle_angles = profile_angles[observed]
    obstacle_ranges = profile_ranges[observed]
    clearances = np.full(candidates.shape, maximum_depth, dtype=np.float32)

    if obstacle_ranges.size:
        for index, heading in enumerate(candidates):
            delta = obstacle_angles - heading
            delta = np.arctan2(np.sin(delta), np.cos(delta))
            forward = obstacle_ranges * np.cos(delta)
            lateral = np.abs(obstacle_ranges * np.sin(delta))
            in_corridor = (forward > 0.0) & (lateral < corridor_half_width)
            if np.any(in_corridor):
                clearances[index] = float(np.min(forward[in_corridor]))

    passable = clearances >= minimum_clearance
    desired_cost = 2.6 * np.abs(candidates - desired)
    continuity_cost = hysteresis_weight * np.abs(candidates - previous)
    steering_cost = 0.18 * np.abs(candidates)
    clearance_reward = 0.45 * np.clip(clearances / maximum_depth, 0.0, 1.0)
    costs = desired_cost + continuity_cost + steering_cost - clearance_reward
    if abs(previous) > 0.12:
        direction_change = candidates * previous < -0.01
        costs[direction_change] += switch_penalty

    if np.any(passable):
        masked_costs = np.where(passable, costs, np.inf)
        selected = int(np.argmin(masked_costs))
        is_passable = True
    else:
        # No translational corridor is safe. Select the best escape bearing,
        # but report it as non-passable so the caller rotates without driving.
        escape_score = clearances - 0.35 * np.abs(candidates - desired)
        escape_score -= 0.20 * np.abs(candidates - previous)
        selected = int(np.argmax(escape_score))
        is_passable = False

    candidate_step = float(candidates[1] - candidates[0])
    left = selected
    right = selected
    while left > 0 and passable[left - 1]:
        left -= 1
    while right + 1 < passable.size and passable[right + 1]:
        right += 1
    angular_width = max(candidate_step, (right - left + 1) * candidate_step)
    probe_distance = min(float(clearances[selected]), 1.0)
    gap_width = (
        2.0 * corridor_half_width +
        2.0 * probe_distance * math.tan(min(angular_width, 1.4) / 2.0)
    )
    return LocalPathPlan(
        heading=float(candidates[selected]),
        clearance=float(clearances[selected]),
        gap_width=float(gap_width),
        passable=is_passable,
    )


def select_exploration_heading(
    profile_angles: np.ndarray,
    profile_ranges: np.ndarray,
    previous_failed_heading: Optional[float] = None,
    robot_width: float = 0.43,
    safety_margin: float = 0.05,
    minimum_clearance: float = 0.75,
    maximum_depth: float = 8.0,
) -> LocalPathPlan:
    """Choose a wide, distant 360-degree opening for viewpoint exploration.

    Unlike the forward local planner, this selector may point behind the robot.
    It is used only when a bay has never been observed or a stored bay pose has
    proved unreachable. A recently stalled heading is penalized so recovery
    does not repeatedly choose the same blocked corridor.
    """
    if (
        profile_angles.size == 0 or
        profile_ranges.size != profile_angles.size
    ):
        return LocalPathPlan(0.0, 0.0, 0.0, False)

    candidates = np.asarray(profile_angles, dtype=np.float32)
    corridor_half_width = robot_width / 2.0 + safety_margin
    observed = np.isfinite(profile_ranges) & (profile_ranges < maximum_depth * 0.995)
    obstacle_angles = profile_angles[observed]
    obstacle_ranges = profile_ranges[observed]
    clearances = np.full(candidates.shape, maximum_depth, dtype=np.float32)

    if obstacle_ranges.size:
        for index, heading in enumerate(candidates):
            delta = obstacle_angles - heading
            delta = np.arctan2(np.sin(delta), np.cos(delta))
            forward = obstacle_ranges * np.cos(delta)
            lateral = np.abs(obstacle_ranges * np.sin(delta))
            in_corridor = (forward > 0.0) & (lateral < corridor_half_width)
            if np.any(in_corridor):
                clearances[index] = float(np.min(forward[in_corridor]))

    passable = clearances >= minimum_clearance
    # Prefer a direction whose neighboring bins are also open. This avoids
    # mistaking one noisy long-range ray for a traversable exploration route.
    normalized = np.clip(clearances / maximum_depth, 0.0, 1.0)
    neighborhood = (
        0.25 * np.roll(normalized, 1) +
        0.50 * normalized +
        0.25 * np.roll(normalized, -1)
    )
    scores = 2.8 * normalized + 1.2 * neighborhood
    # When equally open, use the route requiring the smallest initial turn.
    scores -= 0.12 * np.abs(candidates)

    if previous_failed_heading is not None:
        failed_delta = np.arctan2(
            np.sin(candidates - previous_failed_heading),
            np.cos(candidates - previous_failed_heading),
        )
        scores -= 4.0 * np.exp(-np.square(failed_delta / 0.55))

    if np.any(passable):
        selected = int(np.argmax(np.where(passable, scores, -np.inf)))
        is_passable = True
    else:
        selected = int(np.argmax(scores))
        is_passable = False

    bin_width = 2.0 * math.pi / candidates.size
    run_bins = 1
    if is_passable:
        for offset in range(1, candidates.size):
            if not passable[(selected - offset) % candidates.size]:
                break
            run_bins += 1
        for offset in range(1, candidates.size - run_bins + 1):
            if not passable[(selected + offset) % candidates.size]:
                break
            run_bins += 1
    angular_width = min(1.4, run_bins * bin_width)
    probe_distance = min(float(clearances[selected]), 1.0)
    gap_width = (
        2.0 * corridor_half_width +
        2.0 * probe_distance * math.tan(angular_width / 2.0)
    )
    return LocalPathPlan(
        heading=float(candidates[selected]),
        clearance=float(clearances[selected]),
        gap_width=float(gap_width),
        passable=is_passable,
    )


def find_color_target(
    bgr_image: np.ndarray,
    depth: Optional[np.ndarray],
    color: str,
    minimum_area_fraction: float = 0.004,
    maximum_depth: float = 8.0,
    horizontal_fov: float = 1.22173,
) -> Optional[TargetObservation]:
    """Find the largest saturated delivery-station panel of the requested color."""
    if color not in HSV_RANGES or bgr_image is None or bgr_image.size == 0:
        return None
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in HSV_RANGES[color]:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    image_area = float(mask.shape[0] * mask.shape[1])
    area_fraction = area / image_area
    if area_fraction < minimum_area_fraction:
        return None

    moments = cv2.moments(contour)
    if moments['m00'] <= 0.0:
        return None
    center_x = moments['m10'] / moments['m00']
    horizontal_error = (center_x - mask.shape[1] / 2.0) / (mask.shape[1] / 2.0)

    distance = maximum_depth
    if depth is not None and depth.shape == mask.shape:
        target_ranges = horizontal_ray_ranges(depth, horizontal_fov)
        target_depths = target_ranges[mask > 0]
        valid = target_depths[
            np.isfinite(target_depths) & (target_depths > 0.08) &
            (target_depths <= maximum_depth)
        ]
        if valid.size >= 12:
            distance = float(np.median(valid))

    return TargetObservation(
        horizontal_error=float(np.clip(horizontal_error, -1.0, 1.0)),
        area_fraction=area_fraction,
        distance=distance,
    )
