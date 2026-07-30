# Autonomous Robot Car

A simulation-first ROS 2 Jazzy project for a four-wheel-drive indoor delivery robot. The robot carries a modeled 100 g parcel on its top tray and navigates using only its top-mounted RGB-D camera—there is no LiDAR in the model, bridge, or controller.

The project includes:

- A detailed URDF/Xacro delivery robot with four independently modeled driven wheels
- Skid-steer dynamics, odometry, wheel state publication, camera, tray, and 100 g payload
- A 6 m × 6 m Gazebo room with furniture obstacles and red, green, and blue delivery bays
- Camera-only polar-gap path planning, visual destination search, approach, and arrival detection
- A live browser dashboard with camera video, mission controls, telemetry, and safety state
- RViz robot, TF, odometry-trail, and camera visualization
- Headless mode and automated model/navigation tests

## Sensor design

The simulated sensor is one panoramic RGB-D camera pod with four internal
views: front, left, right, and rear. The front view supplies aligned color and
depth for destination recognition; all four depth views form continuous 360°
camera coverage for clearance checks. This makes metric indoor obstacle
clearance reproducible with a panoramic or multi-lens depth-camera module on
hardware. The system does not publish a laser scan or use LiDAR.

Each view has a 100° horizontal field of view, so adjacent views overlap by
10°. The safety controller uses the dedicated side views as swept-corner
guards during skid turns and does not command reverse motion. If both turning
sides are too close, it enters `TRAPPED` and waits for operator help.

A plain monocular RGB camera cannot directly provide reliable metric obstacle distance without additional scale estimation. If the eventual hardware has only a monocular camera, replace the depth input with a visual-inertial odometry/monocular-depth component and revalidate every stopping distance before operating near people.

## Requirements

- Ubuntu with ROS 2 Jazzy
- Gazebo Harmonic / `ros_gz`
- Python OpenCV and NumPy

Install missing ROS packages with:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-cv-bridge \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-ros-gz \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro \
  python3-opencv python3-numpy
```

## Build

From this directory:

```bash
./build.sh
source install/setup.bash
```

Or build conventionally:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run the full simulation

```bash
./run_sim.sh
```

This opens Gazebo and RViz, starts the front and surround camera bridges, the autonomous controller, and the dashboard. Open the dashboard at:

<http://localhost:8080>

The launcher automatically removes Snap runtime-library paths inherited from
the integrated terminal of a Snap-installed VS Code. This prevents RViz and
Gazebo from accidentally loading `/snap/core20` versions of `libpthread` or
GTK/Qt plugins instead of the Ubuntu/ROS 2 Jazzy versions.

Choose a colored bay and press **Start mission**. Blue is a useful first test because the robot begins facing generally toward the blue side of the room. The central cabinet forces it to demonstrate avoidance. **Autonomous patrol** continuously roams without selecting a delivery marker.

Useful launch variants:

```bash
# Simulation without graphical windows; dashboard remains available
./run_sim.sh headless:=true rviz:=false

# Disable the autonomous controller for manual /cmd_vel testing
./run_sim.sh autonomy:=false

# Disable dashboard or change its port
./run_sim.sh dashboard:=false
./run_sim.sh dashboard_port:=8090

# Inspect only the robot model in RViz
ros2 launch autonomous_robot_car display.launch.py
```

## Dashboard workflow

1. Confirm **ROS CONNECTED**, camera **LIVE**, and collision guard **ACTIVE**.
2. Confirm the simulated reference payload shows **100 g LOADED**.
3. Select Red, Green, Blue, or Patrol and start the mission.
4. Pause/resume as needed. **Stop** always publishes a zero velocity command.
5. At a colored destination, the controller enters `ARRIVED` and stops. Remove the payload in the UI and acknowledge delivery.

The payload toggle represents the delivery workflow; the Gazebo reference payload remains rigidly mounted so the simulated mass and center of gravity are deterministic.

## ROS interfaces

| Topic | Type | Purpose |
|---|---|---|
| `/camera/image` | `sensor_msgs/Image` | RGB navigation and dashboard feed |
| `/camera/depth_image` | `sensor_msgs/Image` | Camera-derived obstacle distance |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration |
| `/camera/{left,right,rear}/image` | `sensor_msgs/Image` | Surround RGB target handoff |
| `/camera/{left,right,rear}/depth_image` | `sensor_msgs/Image` | 360° camera clearance coverage |
| `/mission/command` | `std_msgs/String` | Mission and safety commands |
| `/autonomy/status` | `std_msgs/String` | JSON controller state for dashboard/diagnostics |
| `/cmd_vel` | `geometry_msgs/Twist` | Skid-steer velocity command |
| `/odom` | `nav_msgs/Odometry` | Simulated wheel odometry |
| `/joint_states` | `sensor_msgs/JointState` | Four wheel positions/velocities |
| `/tf` | `tf2_msgs/TFMessage` | `odom` to `base_footprint` transform |

Example command-line mission control:

```bash
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: 'start:blue'}"
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: 'pause'}"
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: 'stop'}"
```

## Controller behavior

The navigator uses a fail-safe state machine:

1. Stop if the RGB/depth camera heartbeat is stale.
2. Fuse the four depth views into a 72-bin panoramic obstacle profile.
3. Run a VFH-style local planner over 33 candidate headings, inflating obstacles by the robot width and safety margin.
4. Detect the selected station using a vivid HSV color mask that rejects muted furniture colors.
5. Route toward each fixed bay's saved approach pose even when its colored panel is completely occluded.
6. Carry the last RGB-D goal observation through short occlusions with wheel odometry while all collision decisions remain camera-based.
7. Move to new viewpoints through wide 360-degree camera gaps when no mapped or remembered goal is available; progress is measured by odometry, and a stalled corridor is not immediately retried.
8. Preserve the selected gap in world coordinates with steering hysteresis, preventing left/right edge oscillation.
9. Reacquire and center the station, then stop at the visual dock marker; reverse is never commanded.

Tuning parameters are in `src/autonomous_robot_car/config/navigation.yaml`. The conservative simulated top speed is 0.45 m/s at the Gazebo drive plugin and 0.22 m/s at the autonomy layer.

The modeled wheel-to-wheel width is approximately 0.43 m. With the default
`gap_safety_margin: 0.05`, the planner requires about 0.53 m of usable gap
width. Reduce that margin only after measuring the physical robot and
calibrating every camera extrinsic; increasing it makes planning more
conservative. Planner telemetry is included in `/autonomy/status` as selected
heading, clearance, and estimated gap width.

The `*_bay_x` and `*_bay_y` parameters are saved approach poses in the
simulation's startup `odom` frame. They model the fixed bay poses that would
normally be recorded after building a visual map on the physical robot. Set
`use_known_bay_poses: false` to test unknown-room exploration instead. Because
the four camera views already cover 360 degrees, the robot does not waste time
spinning at one viewpoint when a bay is hidden; it translates to reveal it.

## Validation

Run unit and package tests:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

For a manual integration check:

```bash
# Terminal 1
./run_sim.sh headless:=true rviz:=false

# Terminal 2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic hz /camera/image
ros2 topic hz /camera/depth_image
ros2 topic echo /autonomy/status
```

Confirm that the front streams update near 15 Hz, the surround streams near 10 Hz, the status reports `camera_ok: true`, odometry changes after starting a mission, and `/cmd_vel` returns to zero after Pause, Stop, camera loss, or Arrival.

### GUI troubleshooting

If RViz or Gazebo was started manually and reports a symbol error involving
`/snap/core20/.../libpthread.so.0`, launch it through `./run_sim.sh`. The script
sanitizes the inherited Snap environment before sourcing ROS. Existing failed
launches should be stopped with `Ctrl+C` before trying again.

`./run_sim.sh` permits only one project simulation at a time and gives every
run an isolated Gazebo transport partition. This prevents two Gazebo servers
from publishing competing `/clock` timelines—the cause of RViz's
`Detected jump back in time. Resetting RViz.` warning. RViz is also delayed
until the simulation clock is stable.

## Moving to hardware

Keep the ROS topic contract and replace the simulation adapters:

| Simulation component | Hardware replacement |
|---|---|
| Gazebo panoramic RGB-D pod | Calibrated panoramic/multi-camera depth drivers with equivalent overlap |
| Gazebo DiffDrive plugin | Motor controller with encoder feedback and watchdog |
| `/odom` from Gazebo | Wheel encoder odometry, preferably fused with visual-inertial odometry |
| Fixed 100 g payload | Load-rated tray plus an optional load cell/interlock |

Before physical operation, measure the real wheel radius/separation, calibrate camera intrinsics and camera-to-base extrinsics, add a hardware emergency stop and velocity watchdog, reduce speed, and retune stopping distances on the actual floor. This demonstration controller is suitable for structured rooms with marked destinations; production deployment among people requires a formally validated safety layer and more robust visual localization.

## Project layout

```text
AUTONOMOUS_ROBOT_CAR/
├── build.sh
├── run_sim.sh
└── src/autonomous_robot_car/
    ├── autonomous_robot_car/   # Navigation and dashboard ROS nodes
    ├── config/                 # Bridge, controller, and RViz configuration
    ├── dashboard/              # Browser UI (no external web dependencies)
    ├── launch/                 # Full simulation and RViz-only launch files
    ├── test/                   # Image-processing and model validation tests
    ├── urdf/                   # Four-wheel robot Xacro
    └── worlds/                 # Gazebo delivery room
```
