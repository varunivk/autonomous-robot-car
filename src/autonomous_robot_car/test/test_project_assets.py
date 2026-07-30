from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).parents[1]


def test_world_is_valid_xml_and_has_three_delivery_stations():
    world = ET.parse(PACKAGE_ROOT / 'worlds' / 'delivery_room.sdf')
    names = {element.attrib.get('name') for element in world.findall('.//model')}
    assert 'red_delivery_station' in names
    assert 'green_delivery_station' in names
    assert 'blue_delivery_station' in names


def test_robot_xacro_expands_with_four_driven_wheels_and_camera():
    model = PACKAGE_ROOT / 'urdf' / 'autonomous_robot_car.urdf.xacro'
    result = subprocess.run(
        ['xacro', str(model)],
        check=True,
        capture_output=True,
        text=True,
    )
    root = ET.fromstring(result.stdout)
    joint_names = {joint.attrib['name'] for joint in root.findall('joint')}
    for position in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        assert f'{position}_wheel_joint' in joint_names
    sensors = root.findall(".//sensor[@type='rgbd_camera']")
    assert len(sensors) == 4
    assert not root.findall(".//sensor[@type='gpu_lidar']")
    assert not root.findall(".//sensor[@type='lidar']")


def test_payload_mass_is_exactly_100_grams():
    model_text = (PACKAGE_ROOT / 'urdf' / 'autonomous_robot_car.urdf.xacro').read_text()
    assert '<link name="payload_100g_link">' in model_text
    assert '<xacro:box_inertial mass="0.100"' in model_text


def test_camera_pod_has_four_views_and_no_lidar():
    model_text = (PACKAGE_ROOT / 'urdf' / 'autonomous_robot_car.urdf.xacro').read_text()
    assert model_text.count('type="rgbd_camera"') == 4
    assert model_text.count('<horizontal_fov>1.74533</horizontal_fov>') == 4
    assert 'lidar' not in model_text.lower()


def test_controller_does_not_reverse_into_camera_blind_spot():
    controller_text = (
        PACKAGE_ROOT / 'autonomous_robot_car' / 'camera_navigator.py'
    ).read_text()
    assert 'return -0.10' not in controller_text
    assert 'command.linear.x = float(linear)' in controller_text
