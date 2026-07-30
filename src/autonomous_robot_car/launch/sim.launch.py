"""Launch the complete delivery-room simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('autonomous_robot_car')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    world = os.path.join(package_share, 'worlds', 'warehouse.sdf')
    model = os.path.join(package_share, 'urdf', 'autonomous_robot_car.urdf.xacro')
    bridge_config = os.path.join(package_share, 'config', 'bridge.yaml')
    navigation_config = os.path.join(package_share, 'config', 'navigation.yaml')
    rviz_config = os.path.join(package_share, 'config', 'autonomous_robot_car.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    robot_description = ParameterValue(
        Command(['xacro', ' ', model]),
        value_type=str,
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        condition=UnlessCondition(headless),
        launch_arguments={
            'gz_args': f'-r {world}',
            'on_exit_shutdown': 'true',
        }.items(),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        condition=IfCondition(headless),
        launch_arguments={
            'gz_args': f'-s -r {world}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'delivery_bot',
            '-topic', 'robot_description',
            '-x', '-1.5', '-y', '0.0', '-z', '0.015',
            '-Y', '0.0',
        ],
        output='screen',
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': use_sim_time}],
        output='screen',
    )
    rgb_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='rgb_image_bridge',
        arguments=['/camera/image'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    depth_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='depth_image_bridge',
        arguments=['/camera/depth_image'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    left_depth_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='left_depth_image_bridge',
        arguments=['/camera/left/depth_image'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    right_depth_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='right_depth_image_bridge',
        arguments=['/camera/right/depth_image'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    rear_depth_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='rear_depth_image_bridge',
        arguments=['/camera/rear/depth_image'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    left_rgb_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='left_rgb_image_bridge',
        arguments=['/camera/left/image'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    right_rgb_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='right_rgb_image_bridge',
        arguments=['/camera/right/image'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    rear_rgb_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='rear_rgb_image_bridge',
        arguments=['/camera/rear/image'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    navigator = Node(
        package='autonomous_robot_car',
        executable='camera_navigator',
        condition=IfCondition(LaunchConfiguration('autonomy')),
        parameters=[navigation_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )
    dashboard = Node(
        package='autonomous_robot_car',
        executable='dashboard',
        condition=IfCondition(LaunchConfiguration('dashboard')),
        parameters=[{
            'use_sim_time': use_sim_time,
            'host': LaunchConfiguration('dashboard_host'),
            'port': LaunchConfiguration('dashboard_port'),
        }],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    stable_clock_rviz = TimerAction(
        period=3.0,
        actions=[rviz],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('autonomy', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('dashboard', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('dashboard_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('dashboard_port', default_value='8080'),
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rgb_bridge,
        depth_bridge,
        left_depth_bridge,
        right_depth_bridge,
        rear_depth_bridge,
        left_rgb_bridge,
        right_rgb_bridge,
        rear_rgb_bridge,
        navigator,
        dashboard,
        stable_clock_rviz,
    ])
