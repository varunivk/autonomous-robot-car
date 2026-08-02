# Autonomous Robot Car

A simulation-first ROS 2 Jazzy project for a four-wheel-drive indoor delivery robot. The robot carries a modeled 100 g parcel on its delivery tray and navigates using only its cameras — there is no LiDAR in the model, bridge, or controller.

The project includes:

- A detailed URDF/Xacro delivery robot with four independently modeled driven wheels
- Skid-steer (diff-drive) dynamics, odometry, wheel state publication, camera pod, tray, and 100 g payload
- A Gazebo warehouse world with shelving, a charging station, obstacles, and red, green, and blue delivery bays (plus a smaller `delivery_room.sdf` test world)
- Camera-only polar-gap path planning, visual destination search, approach, and arrival detection
- A live browser dashboard with camera video, mission controls, telemetry, and safety state
- RViz robot, TF, and camera visualization
- Headless mode and automated model/navigation tests

## Sensor design

The simulated sensor is one camera pod with four `rgbd_camera` views: front, left, right, and rear. The front view supplies aligned color and depth for destination recognition; all four depth views together give continuous 360° camera coverage for clearance checks. Each view runs at 10 Hz with a 100° (1.74533 rad) horizontal field of view and an 8 m depth range.

The safety controller uses the dedicated left/right/rear views as swept-body guards during in-place pivots. If both turning sides — or the space behind — are too tight, it stops and enters `TRAPPED`; if it's blocked for too long it backs straight out, but only once the rear view confirms clear room behind it.

No laser scanner is modeled, bridged, or subscribed to anywhere in the stack — `test_camera_pod_has_four_views_and_no_lidar` asserts this directly against the URDF.

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

This opens Gazebo (loading `worlds/warehouse.sdf`) and RViz, starts the front and surround camera bridges, the autonomous controller, and the dashboard. Open the dashboard at:

<http://localhost:8080>

The launcher automatically strips Snap runtime-library paths (`LD_LIBRARY_PATH`, `PYTHONPATH`, `QT_PLUGIN_PATH`, etc.) that a Snap-installed VS Code's integrated terminal can inherit. This prevents RViz and Gazebo from loading `/snap` versions of `libpthread`/glibc or GTK/Qt plugins instead of the host Ubuntu/ROS 2 Jazzy ones.

Choose a colored bay and press **Start mission**. **Autonomous patrol** continuously roams with obstacle avoidance and no delivery marker selected.

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

1. Confirm the connection badge is online, the camera badge is live, and the depth collision guard reads **ACTIVE**.
2. Confirm the payload panel shows **100 g LOADED**.
3. Select Red, Green, Blue, or Patrol and start the mission.
4. Pause/resume as needed. **Stop** always publishes a zero velocity command.
5. At a colored destination, the controller enters `ARRIVED` and stops. Toggle the payload off in the UI and acknowledge delivery to reset the mission.

The payload toggle is a UI/telemetry indicator only — the Gazebo `payload_100g_link` remains rigidly mounted to the tray at all times, so the simulated mass and center of gravity stay deterministic regardless of what the dashboard shows.

## ROS interfaces

| Topic | Type | Purpose |
|---|---|---|
| `/camera/image` | `sensor_msgs/Image` | Front RGB — navigation and dashboard feed |
| `/camera/depth_image` | `sensor_msgs/Image` | Front camera-derived obstacle distance |
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

The navigator (`camera_navigator`) runs a fail-safe state machine at 15 Hz (`command_rate`):

1. Stop if the front or any surround camera's RGB/depth heartbeat is stale.
2. Fuse the four depth views into a panoramic obstacle profile (72 bins by default).
3. Run a polar-gap local planner over 33 candidate headings, inflating obstacles by the robot's width and a safety margin.
4. Detect the selected station using an HSV color mask, filtered by contour area and morphological open/close to reject small or muted-color noise.
5. Route toward each bay's saved approach pose (`config/navigation.yaml`) even when its colored panel is fully occluded, if `use_known_bay_poses` is enabled.
6. Carry the last RGB-D goal observation through short occlusions using wheel odometry, while all collision decisions stay camera-based.
7. Move to new viewpoints through wide camera-confirmed gaps when no mapped or remembered goal is available; progress is measured by odometry, and a stalled route is not immediately retried.
8. Preserve the selected gap in world coordinates with steering hysteresis and a switch penalty, preventing left/right edge oscillation.
9. Center on the station once visible, then stop at the arrival distance; reverse is only ever used for stuck recovery, never as a normal approach behavior.

Tuning parameters live in `src/autonomous_robot_car/config/navigation.yaml`. The Gazebo `DiffDrive` plugin caps the robot at 0.45 m/s forward / 0.20 m/s reverse; the autonomy layer's own speeds (cruise, search, approach) are lower still and are set per-mode in that config file.

The modeled wheel track (`wheel_separation`) and the `robot_width` planner parameter are both roughly 0.43 m. With the default `gap_safety_margin: 0.05`, the planner requires about 0.48 m of usable gap width. Planner telemetry (selected heading, clearance, gap width) is published in `/autonomy/status`.

The `*_bay_x` / `*_bay_y` parameters are saved approach poses in the simulation's startup `odom` frame, matching `worlds/warehouse.sdf`. They model the fixed bay poses that would normally be recorded after building a visual map on the physical robot. Set `use_known_bay_poses: false` to force pure visual exploration instead.

## Validation

Run unit and package tests:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

`test_navigation_logic.py` covers depth-sector/obstacle detection, ray-range conversion, the polar gap planner, exploration heading selection, and color-target detection. `test_project_assets.py` validates the world files (three delivery stations by model name), confirms the xacro model expands to four driven wheels and exactly four `rgbd_camera` sensors with no LiDAR, confirms the 100 g payload link/mass, and checks the controller source for a specific reversing regression.

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

Confirm that the camera streams update near 10 Hz, the status reports `camera_ok: true`, odometry changes after starting a mission, and `/cmd_vel` returns to zero after Pause, Stop, camera loss, or Arrival.

### GUI troubleshooting

If RViz or Gazebo was started manually (not through `./run_sim.sh`) and reports a symbol error involving a Snap library path (e.g. `libpthread.so.0` under `/snap/...`), launch it through `./run_sim.sh` instead — the script sanitizes the inherited Snap environment before sourcing ROS. Stop any existing failed launch with `Ctrl+C` before trying again.

`./run_sim.sh` allows only one simulation per user at a time (via a lock file) and gives each run an isolated Gazebo transport partition (`GZ_PARTITION`). This prevents two Gazebo servers from publishing competing `/clock` timelines. RViz is also delayed by a few seconds at startup until the simulation clock is stable.

## Moving to hardware

Keep the ROS topic contract and replace the simulation adapters:

| Simulation component | Hardware replacement |
|---|---|
| Gazebo `rgbd_camera` pod (4 views) | Calibrated panoramic/multi-camera depth drivers with equivalent overlap |
| Gazebo `DiffDrive` plugin | Motor controller with encoder feedback and a velocity watchdog |
| `/odom` from Gazebo | Wheel encoder odometry, ideally fused with visual-inertial odometry |
| Fixed 100 g payload | Load-rated tray plus an optional load cell/interlock |
| `red/green/blue_bay_x/y` fixed poses | Poses recorded from an actual visual map, or `use_known_bay_poses: false` |

Before physical operation, measure the real wheel radius/separation, calibrate camera intrinsics and camera-to-base extrinsics, add a hardware emergency stop, reduce speed limits, and retune stopping distances (`emergency_distance`, `stop_distance`, `arrival_distance`) on the actual floor. This controller is a demonstration suited to structured rooms with marked destinations; deployment among people needs a formally validated safety layer.

## Project layout

```text
autonomous-robot-car/
├── build.sh
├── run_sim.sh
└── src/autonomous_robot_car/
    ├── autonomous_robot_car/   # camera_navigator, navigation_logic, dashboard_node
    ├── config/                 # Bridge, controller (navigation.yaml), and RViz configuration
    ├── dashboard/              # Browser UI (no external web dependencies)
    ├── launch/                 # sim.launch.py, display.launch.py
    ├── test/                   # Navigation-logic unit tests and project-asset validation
    ├── urdf/                   # Four-wheel robot Xacro
    └── worlds/                 # warehouse.sdf (default sim world), delivery_room.sdf
```
