from autonomous_robot_car.navigation_logic import (
    find_color_target,
    horizontal_ray_ranges,
    measure_depth_sectors,
    normalize_depth,
    panoramic_depth_profile,
    select_exploration_heading,
    select_local_path,
)
import cv2
import numpy as np


def test_depth_is_converted_from_millimetres():
    raw = np.array([[500, 1500, 0]], dtype=np.uint16)
    depth = normalize_depth(raw, '16UC1')
    assert depth[0, 0] == 0.5
    assert depth[0, 1] == 1.5
    assert np.isnan(depth[0, 2])


def test_depth_sectors_detect_center_obstacle():
    depth = np.full((100, 150), 4.0, dtype=np.float32)
    depth[40:88, 65:85] = 0.55
    sectors = measure_depth_sectors(depth)
    assert sectors.center < 0.6
    assert sectors.left > 3.5
    assert sectors.right > 3.5


def test_depth_sectors_detect_narrow_obstacle():
    depth = np.full((120, 200), 5.0, dtype=np.float32)
    depth[36:75, 97:103] = 0.68
    sectors = measure_depth_sectors(depth)
    assert sectors.center < 0.7


def test_wide_camera_axis_depth_is_converted_to_ray_range():
    depth = np.ones((10, 100), dtype=np.float32)
    ranges = horizontal_ray_ranges(depth, 2.61799)
    assert ranges[5, 0] > 3.5
    assert 0.99 < ranges[5, 50] < 1.02


def test_red_target_has_horizontal_error_and_depth():
    image = np.zeros((120, 200, 3), dtype=np.uint8)
    cv2.rectangle(image, (125, 30), (185, 105), (0, 0, 255), thickness=-1)
    depth = np.full((120, 200), 3.0, dtype=np.float32)
    depth[30:106, 125:186] = 1.25
    observation = find_color_target(image, depth, 'red')
    assert observation is not None
    assert observation.horizontal_error > 0.3
    # Off-axis radial range is correctly longer than the 1.25 m optical depth.
    assert 1.3 < observation.distance < 1.4


def test_tiny_color_patch_is_ignored():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    image[2:4, 2:4] = (255, 0, 0)
    assert find_color_target(image, None, 'blue') is None


def test_muted_blue_furniture_is_not_a_delivery_target():
    image = np.full((100, 160, 3), (128, 97, 64), dtype=np.uint8)
    depth = np.full((100, 160), 1.0, dtype=np.float32)

    assert find_color_target(image, depth, 'blue') is None


def test_panoramic_profile_places_left_camera_obstacle_at_positive_yaw():
    depth = np.full((100, 160), 8.0, dtype=np.float32)
    depth[34:45, 75:85] = 0.8

    angles, ranges = panoramic_depth_profile([(depth, 1.74533, np.pi / 2.0)])
    nearest = int(np.argmin(np.abs(angles - np.pi / 2.0)))

    assert ranges[nearest] < 0.85


def test_local_planner_accepts_gap_wider_than_inflated_robot():
    angles = np.linspace(-np.pi, np.pi, 72, endpoint=False, dtype=np.float32)
    ranges = np.full(72, 8.0, dtype=np.float32)
    for edge in (-0.70, 0.70):
        ranges[int(np.argmin(np.abs(angles - edge)))] = 0.50

    plan = select_local_path(angles, ranges, desired_heading=0.0)

    assert plan.passable
    assert abs(plan.heading) < 0.10
    assert plan.gap_width > 0.53


def test_local_planner_does_not_enter_gap_narrower_than_robot_margin():
    angles = np.linspace(-np.pi, np.pi, 72, endpoint=False, dtype=np.float32)
    ranges = np.full(72, 8.0, dtype=np.float32)
    for edge in (-0.22, 0.22):
        ranges[int(np.argmin(np.abs(angles - edge)))] = 0.50

    plan = select_local_path(angles, ranges, desired_heading=0.0)

    assert plan.passable
    assert abs(plan.heading) > 0.10


def test_local_planner_hysteresis_keeps_side_at_symmetric_edge():
    angles = np.linspace(-np.pi, np.pi, 72, endpoint=False, dtype=np.float32)
    ranges = np.full(72, 8.0, dtype=np.float32)
    ranges[int(np.argmin(np.abs(angles)))] = 0.42

    plan = select_local_path(
        angles,
        ranges,
        desired_heading=0.0,
        previous_heading=0.50,
    )

    assert plan.passable
    assert plan.heading > 0.0


def test_exploration_selects_wide_side_route_instead_of_blocked_front():
    angles = np.linspace(-np.pi, np.pi, 72, endpoint=False, dtype=np.float32)
    ranges = np.full(72, 2.5, dtype=np.float32)
    ranges[np.abs(angles) < 0.55] = 0.45
    ranges[np.abs(angles - np.pi / 2.0) < 0.50] = 8.0

    plan = select_exploration_heading(angles, ranges)

    assert plan.passable
    assert 0.70 < plan.heading < 2.20


def test_exploration_does_not_retry_recently_stalled_heading():
    angles = np.linspace(-np.pi, np.pi, 72, endpoint=False, dtype=np.float32)
    ranges = np.full(72, 1.8, dtype=np.float32)
    left = np.abs(angles - np.pi / 2.0) < 0.45
    right = np.abs(angles + np.pi / 2.0) < 0.45
    ranges[left | right] = 8.0

    plan = select_exploration_heading(
        angles,
        ranges,
        previous_failed_heading=np.pi / 2.0,
    )

    assert plan.passable
    assert plan.heading < -0.70
