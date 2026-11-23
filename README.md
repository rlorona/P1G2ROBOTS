# Proyecto\_RM — Ackermann-Steered Mobile Robot

ROS 2 Jazzy + Gazebo Harmonic simulation of the `Proyecto_RM` robot, including a
parametric URDF/Xacro model, a `ros2_control` hardware interface, and a
trajectory-tracking controller that drives the robot along a Christmas-tree path
(trunk, three rounded branch tiers and a five-point star on top).

The robot was designed in Fusion 360. The CAD source (`cad/original/Proyecto_RM.f3d`)
and the STL meshes (`ros2_ws/src/proyecto_rm_description/meshes/`) are included in
the repository; the corrected dimensioned drawings are in `docs/drawings/`
(`Assembly.pdf`, `BASE_LINK.pdf`, `CAMERA.pdf`, `Wheel.pdf`).

---

## 1. Repository layout

```
.
├── cad/original/                 Fusion 360 source and CAD metadata (F3D, description package)
├── docs/
│   ├── paper/                    IEEE conference paper (LaTeX sources + PDF)
│   ├── drawings/                 Dimensioned part drawings (PDF) — plans
│   ├── figures/                  Figures used in the paper (PDF)
│   ├── results/                  Simulation results (CSV, summary, figures)
│   ├── video/                    Trajectory video (MP4)
│   └── presentation/             Final presentation (PPTX)
├── ros2_ws/
│   └── src/
│       ├── proyecto_rm_description/   URDF/Xacro, STL meshes, controllers.yaml
│       ├── proyecto_rm_control/       Trajectory-tracking controller (Python)
│       └── proyecto_rm_bringup/       Launch files, Gazebo world, RViz config
└── scripts/
    ├── generate_track_world.py   Regenerates the painted Christmas-tree world
    └── make_figures.py            Generates the paper figures from the results
```

## 2. Requirements

- Ubuntu 24.04
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic (Gazebo Sim 8)
- Python 3.12 with NumPy

Install the dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui \
  python3-numpy
```

`ros-jazzy-ros2-controllers` provides the `position_controllers` and
`forward_command_controller` packages used by the low-level controller.
`ros-jazzy-gz-ros2-control` provides the Gazebo `ros2_control` system plugin.

## 3. Build

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y   # optional
colcon build --symlink-install
source install/setup.bash
```

## 4. Run the complete demo

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch proyecto_rm_bringup demo.launch.py
```

This single command:

1. starts Gazebo with the `ackermann_track` world (Christmas tree painted on the
   floor),
2. starts `robot_state_publisher` and spawns the robot at the origin,
3. loads `joint_state_broadcaster`, `steering_controller` and
   `rear_drive_controller`,
4. opens RViz with the robot model, TF tree, reference path and driven path,
5. after 13 s starts the `trajectory_tracker` node.

The robot waits 2 s and then follows the Christmas-tree trajectory for 3 laps. When it
finishes, the controller stops the wheels and writes the results to
`~/.ros/proyecto_rm/`.

### Headless (no GUI)

```bash
ros2 launch proyecto_rm_bringup simulation.launch.py headless:=true
```

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch proyecto_rm_bringup trajectory.launch.py num_laps:=3 max_speed:=0.25
```

### Visualize the model only

```bash
ros2 launch proyecto_rm_bringup display.launch.py
```

Use the joint-state slider panel to move the steering knuckles and the wheels.

## 5. Trajectory tracker

The node `proyecto_rm_control/trajectory_tracker` implements:

- a Christmas-tree outline with a five-point star on top or an ellipse (oval) path,
- pure-pursuit lateral control with speed-dependent lookahead,
- Ackermann steering-angle computation for the inner and outer front wheels,
- speed scheduling from lateral-acceleration and steering-rate limits,
- wheel-odometry integration from the rear-wheel encoders and the virtual IMU
  yaw rate,
- CSV logging of the reference, the estimated pose, the ground-truth pose and
  the cross-track error.

### Launch arguments

| Argument      | Default  | Description                          |
|---------------|----------|--------------------------------------|
| `trajectory`  | `tree`   | `tree` or `oval`                    |
| `max_speed`   | `0.40`   | Maximum linear speed [m/s]           |
| `num_laps`    | `3`      | Number of laps before stopping       |
| `use_sim_time`| `true`   | Use the Gazebo clock                 |

### Main parameters (`proyecto_rm_bringup/config/trajectory.yaml`)

| Parameter            | Value       | Description                              |
|----------------------|-------------|------------------------------------------|
| `wheelbase`          | 0.160 m     | Distance between front and rear axles    |
| `track_width`        | 0.202 m     | Distance between left and right wheels   |
| `wheel_radius`       | 0.037 m     | Wheel radius                             |
| `max_steering_angle` | 30°         | Steering-knuckle limit                   |
| `max_speed`          | 0.40 m/s    | Maximum linear speed                     |
| `max_lateral_accel`  | 0.50 m/s²   | Lateral-acceleration limit               |
| `max_steering_rate`  | 1.50 rad/s  | Steering-rate limit                      |
| `lookahead_min`      | 0.14 m      | Minimum pure-pursuit lookahead          |
| `lookahead_gain`     | 0.10        | Lookahead gain (speed dependent)       |
| `lookahead_max`      | 0.24 m      | Maximum pure-pursuit lookahead          |
| `odom_scale`         | 0.97        | Wheel-odometry calibration factor      |
| `trajectory_type`    | `tree`      | `tree` (Christmas tree) or `oval`      |

The Christmas-tree reference is realised as a G1 chain of line segments and
tangent circular arcs, so all corners are rounded with radii below the
kinematic limit of the robot. The branch tips are rounded with a radius that
saturates the steering near the $\pm 30^\circ$ limit, which is where the maximum
steering angle of the run is observed.

## 6. Results

After a run the controller writes:

- `~/.ros/proyecto_rm/results.csv` — time history of poses, commands and errors,
- `~/.ros/proyecto_rm/summary.txt` — path length, laps, cross-track RMSE/mean/max
  and odometry drift.

Copy them to `docs/results/` for the report:

```bash
cp ~/.ros/proyecto_rm/results.csv docs/results/
cp ~/.ros/proyecto_rm/summary.txt docs/results/
```

Final results of the 3-lap Christmas-tree run (Gazebo Sim 8, ROS 2 Jazzy):

| Metric | Value |
|---|---|
| Path length | 19.9 m |
| Maximum steering angle | 30.0 deg |
| Cross track error mean | 22.6 mm |
| Cross track error RMS | 41.8 mm |
| Cross track error maximum | 238 mm |
| Heading error maximum | 3.0 deg |
| Odometry drift RMS | 109 mm |
| Odometry drift final | 132 mm |

## 7. Deliverables

Items submitted for the final delivery:

1. **Repository (GitHub/GitLab):** source code of the trajectory and the
   controller, with this `README.md` containing the exact steps to build,
   run the simulation and the dependencies/requirements (ROS 2 Jazzy).
2. **Technical document:** IEEE format (two-column, English) including the
   Abstract, the theoretical analysis, the mathematical model, the design
   parameters and the results obtained.
3. **Assembly plans:** all the bounded drawings of each robot part — see
   `docs/drawings/`.
4. **Simulation video:** clear recording where the robot executes the designed
   trajectory in Gazebo — see `docs/video/trajectory.mp4`.
5. **Presentation video:** recording exposing the project with the slides
   (max 5 min; all participants must appear). Video:
   <https://drive.google.com/file/d/1G0tbty7wQbL91o5IQz3c9XeGfS2OGyxg/view?usp=sharing>
6. **Presentation:** max 10 numbered slides (mathematical model, design
   parameters, graphs/videos of the results, conclusions and recommendations).

## 8. Model notes

- **Frames.** The Fusion 360 model has the camera at `-X`, so the CAD frame is
  kept as `base_link` and a `base_footprint` frame is attached at the ground
  contact point, rotated 180° about Z. All control and navigation quantities are
  expressed in `base_footprint` (+X forward, +Y left, +Z up, REP-103).
- **Steering.** The CAD model has no physical steering mechanism. Two virtual
  revolute knuckles (`front_left_steering_joint`,
  `front_right_steering_joint`) rotate the front wheels about Z within ±30°.
  The rear wheels are driven; the front wheels are passive.
- **Geometry.** Wheelbase 0.160 m, track 0.202 m, wheel radius 0.037 m, body
  0.240 × 0.180 × 0.057 m, total mass ≈ 13.8 kg. Values were measured from the
  STL meshes, which are the authoritative geometry.
- **Collisions.** The chassis uses a box collision, the wheels use cylinder
  collisions, and the camera uses a box. Self-collision is disabled between
  adjacent links to avoid contact artefacts.
- **ros2_control.** The Gazebo system plugin (`gz_ros2_control/GazeboSimSystem`)
  exposes position commands for the two steering joints, velocity commands for
  the two rear wheels, and state interfaces for all six joints.
- **Ground truth.** The `gz-sim-odometry-publisher-system` plugin publishes the
  true robot pose on `/model/proyecto_rm/odometry`, bridged to
  `/ground_truth/odometry`. The IMU sensor (`imu_link`) publishes the yaw rate
  used by the odometry.

## 9. Regenerating the Gazebo track

```bash
python3 scripts/generate_track_world.py
```

The script writes `ros2_ws/src/proyecto_rm_bringup/worlds/ackermann_track.sdf`.
It uses the same parametric curve as the controller, so the painted line and
the reference path always coincide.

## 10. Recording the trajectory video

The world includes a static top-down camera at `top_camera` with the image
streamed on `/top_camera/image_raw` through the `ros_gz_image` bridge.

### View the camera stream

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch proyecto_rm_bringup simulation.launch.py headless:=true
# en otra terminal:
ros2 run rqt_image_view /top_camera/image_raw
```

### Record frames and build the video

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch proyecto_rm_bringup simulation.launch.py headless:=true

python3 scripts/record_camera.py frames/ 5
ffmpeg -y -f concat -safe 0 -i frames/concat.txt -vsync vfr -pix_fmt yuv420p -movflags +faststart trajectory.mp4
```

Place the final video in the repository (for example
`docs/video/trajectory.mp4`) before pushing.

### Watch the recorded trajectory video

The recorded trajectory-tracking run is available here:

[Proyecto\_RM trajectory run — Google Drive](https://drive.google.com/file/d/1G0tbty7wQbL91o5IQz3c9XeGfS2OGyxg/view?usp=sharing)

## 11. Drawings

Dimensioned part drawings exported from Fusion 360 are stored in
`docs/drawings/`. See `docs/drawings/README.md` for the list of sheets and the
dimensions they must contain. The corrected drawings delivered by the team are
in `docs/drawings/`: `Assembly.pdf`, `BASE_LINK.pdf`, `CAMERA.pdf` and
`Wheel.pdf`.

The original assembly drawing (`Proyecto_RM Dibujo_.pdf`) was removed from the
repository: its dimensions did not match the exported STL geometry, so the
corrected drawings in `docs/drawings/` are the authoritative ones.

## 12. Paper

The IEEE conference paper sources are in `docs/paper/`:

```bash
cd docs/paper
pdflatex Paper
bibtex Paper
pdflatex Paper
pdflatex Paper
```

The resulting `docs/paper/Paper.pdf` is a double column, English, conference
format IEEE paper with an abstract and keywords.

## 13. Authors

- Ricardo Lorona
- Alejandro Malucin
- Augusto Pacheco

Escuela Superior Politécnica del Litoral (ESPOL) — Mechatronics Engineering,
Guayaquil, Ecuador.

