# Edge-Avoiding Table Robot — ROS 2 Jazzy + Gazebo Harmonic

A simulated two-wheeled robot that drives around on top of a table and **avoids falling off the edge**.
A downward-tilted lidar on the front of the robot watches the table surface. When the floor "drops away"
in front of it, the robot reverses and turns away from the edge.

The whole project runs inside a **VS Code Dev Container** (Docker, Ubuntu 24.04, ROS 2 Jazzy). The
simulator (Gazebo) and visualizer (RViz) are shown in your web browser through a built-in virtual desktop,
so nothing needs to be installed on your own computer except Docker and VS Code.

![stack](https://img.shields.io/badge/ROS_2-Jazzy-blue) ![sim](https://img.shields.io/badge/Gazebo-Harmonic-orange) ![os](https://img.shields.io/badge/Ubuntu-24.04-E95420)

---

## Contents

1. [How the application works](#1-how-the-application-works)
2. [Folder structure](#2-folder-structure)
3. [What each file does](#3-what-each-file-does)
4. [Setting up VS Code and the Ubuntu/ROS container](#4-setting-up-vs-code-and-the-ubunturos-container)
5. [Building and running](#5-building-and-running)
6. [Viewing the simulation](#6-viewing-the-simulation)
7. [Useful commands](#7-useful-commands)
8. [Troubleshooting](#8-troubleshooting)
9. [Project history: what was done](#9-project-history-what-was-done)

---

## 1. How the application works

```
                ┌──────────────────────── Gazebo (simulator) ────────────────────────┐
                │  world: table made of coffee-table tiles                            │
                │  robot: diff-drive base + downward-tilted lidar                     │
                │  gz_ros2_control plugin ──► controller_manager (inside Gazebo)      │
                └───────┬───────────────────────────────▲─────────────────────────────┘
          /clock, /scan │ (ros_gz_bridge)               │ wheel velocity commands
                        ▼                               │
   ┌──────────────────────────┐   /wheel_controller/cmd_vel   ┌─────────────────────────┐
   │ edge_detection (Python)  │ ─────────────────────────────►│ wheel_controller         │
   │ reads /scan, decides:    │       (TwistStamped)          │ (diff_drive_controller)  │
   │ drive forward or reverse │                               │ publishes /odom + TF     │
   └──────────────────────────┘                               └─────────────────────────┘
   ┌──────────────────────────┐
   │ scan_rays (Python)       │ ─► /scan_rays marker ─► RViz        (green beam lines)
   │ draws the lidar beams    │ ─► Gazebo /marker service ─► Gazebo (green beam lines)
   └──────────────────────────┘
   robot_state_publisher: publishes the robot model (/robot_description) and its TF frames
```

**The edge-detection logic** (in `edge_detection.py`):

- The lidar has 10 beams spread over 1 radian, pointing forward and **down** at the table.
  - Beam index 2 ("down", about 45° below horizontal) normally hits the table about 0.2 m away.
  - Beam index 6 ("up", about 19° below horizontal) normally hits the table further ahead.
- If the "down" beam suddenly measures **more than 0.50 m**, the table surface has disappeared, so an edge has been detected.
- While an edge is detected and the "up" beam reads **more than 0.60 m**, the robot **reverses at 0.4 m/s and turns** (−1.65 rad/s).
- Otherwise it **drives forward at 0.3 m/s**.
- Commands are sent 10 times per second to the `wheel_controller`.

---

## 2. Folder structure

```
robot-project/                         ← the ROS 2 workspace root (open this folder in VS Code)
├── README.md                          ← this file
├── start_desktop.sh                   ← starts the browser-viewable virtual desktop
├── .devcontainer/
│   ├── Dockerfile                     ← the Ubuntu + ROS 2 image, with all dependencies
│   └── devcontainer.json              ← tells VS Code how to build/start the container
├── .vscode/
│   └── settings.json                  ← VS Code workspace settings
│
├── src/                               ← ALL SOURCE CODE lives here (4 ROS 2 packages)
│   ├── bot_bringup/                   ← "start everything" package
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── launch/
│   │       └── simulated_robot.launch.py   ← ★ MAIN ENTRY POINT
│   │
│   ├── bot_description/               ← what the robot and world look like
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── urdf/                      ← robot model (xacro/URDF)
│   │   │   ├── bot.urdf.xacro         ← top-level file, includes the 4 below
│   │   │   ├── bot_base.xacro         ← body, wheels, castors
│   │   │   ├── bot_sensors.xacro      ← lidar mount + laser_frame
│   │   │   ├── gazebo.xacro           ← friction, lidar sensor, Gazebo plugins
│   │   │   └── ros2_control.xacro     ← which joints the controllers drive
│   │   ├── launch/
│   │   │   ├── gazebo.launch.py       ← starts Gazebo, spawns the robot, ROS↔Gazebo bridge
│   │   │   └── display.launch.py      ← view the robot model alone in RViz (no physics)
│   │   ├── worlds/
│   │   │   ├── empty_table.world      ← default world: a table made of coffee tables
│   │   │   └── empty.world            ← plain ground plane
│   │   ├── models/
│   │   │   └── aws_robomaker_residential_CoffeeTable_01/  ← 3D table model used by the world
│   │   ├── rviz/
│   │   │   └── display.rviz           ← saved RViz layout
│   │   └── config/
│   │       └── gui.config             ← Gazebo window layout (starting camera position)
│   │
│   ├── bot_controller/                ← how the wheels are driven
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── config/
│   │   │   └── bot_controllers.yaml   ← diff-drive + joint-state controller settings
│   │   └── launch/
│   │       └── controller.launch.py   ← loads/activates the controllers
│   │
│   └── bot_script/                    ← the robot's "brain" (Python nodes)
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/bot_script        ← empty marker file required by ROS 2 Python packages
│       └── bot_script/
│           ├── __init__.py
│           ├── edge_detection.py      ← edge-avoidance behaviour
│           └── scan_rays.py           ← draws the lidar beams in RViz and Gazebo
│
├── build/   ┐
├── install/ ├─ GENERATED by `colcon build`. Never edit by hand; safe to delete and rebuild.
└── log/     ┘
```

> **Leftover folders:** `src/bot_description/` also contains old `build/`, `install/` and `log/` folders
> (and empty `include/`, `src/` folders in some packages). They come from an earlier build run in the wrong
> directory. Colcon ignores them and they are safe to delete.

---

## 3. What each file does

### Container and editor setup

| File | Role |
|---|---|
| `.devcontainer/Dockerfile` | Builds the development image from `ros:jazzy-ros-base` (Ubuntu 24.04). Installs colcon, rosdep, Gazebo Harmonic (`ros_gz_*`), `ros2_control` and its controllers, `gz_ros2_control`, RViz, xacro, state publishers, and the virtual-desktop tools (TigerVNC, noVNC, fluxbox). |
| `.devcontainer/devcontainer.json` | Tells VS Code to build that image, mount this folder at `/workspaces/robot-project`, set `DISPLAY=:1` so GUI apps open on the virtual desktop, forward port **6080** (the browser desktop), run `start_desktop.sh` every time the container starts, and install the ROS, Python and C++ VS Code extensions. |
| `.vscode/settings.json` | Workspace setting for the CMake Tools extension. |
| `start_desktop.sh` | Starts the virtual desktop: TigerVNC X server on display `:1`, the fluxbox window manager (opens windows maximized), and websockify/noVNC on port 6080. Safe to run repeatedly because it only starts what isn't already running. |

### `bot_bringup`: starts the whole application

| File | Role |
|---|---|
| `launch/simulated_robot.launch.py` | **Main entry point.** Includes `gazebo.launch.py` (simulator + robot) and `controller.launch.py` (wheel controllers), then starts RViz, the `edge_detection` node and the `scan_rays` node. Argument: `world_name` (default `empty_table`). |
| `package.xml` | Declares that this package needs `bot_description`, `bot_controller`, `bot_script` and `rviz2` at run time. |
| `CMakeLists.txt` | Installs the `launch/` folder. |

### `bot_description`: robot model, worlds, visualization

| File | Role |
|---|---|
| `urdf/bot.urdf.xacro` | Top-level robot description; just includes the four files below. |
| `urdf/bot_base.xacro` | Physical robot: `base_footprint` → `base_link` (red 0.7×0.5×0.2 m box, 5 kg), two blue drive wheels (radius 0.15 m, 0.55 m apart) on continuous joints, and front/back black spherical castors. |
| `urdf/bot_sensors.xacro` | Green lidar housing `laser_link` at the front of the robot, plus `laser_frame`, a frame rotated 90° to match the sensor's tilt. Scans are reported in `laser_frame`. |
| `urdf/gazebo.xacro` | Gazebo-only settings: high wheel friction and low castor friction. Defines the **lidar sensor** (`gpu_lidar`: 10 samples from −1.0 to 0 rad, 0.05–4 m range, 5 Hz, tilted 90° so it scans downward, topic `scan`). Loads the `gz_ros2_control` plugin (runs the controllers inside Gazebo using `bot_controllers.yaml`) and the Sensors system (renders the lidar). |
| `urdf/ros2_control.xacro` | Tells `ros2_control` that the hardware is `gz_ros2_control/GazeboSimSystem` and that both wheel joints take velocity commands (±4.5 rad/s) and report position and velocity. |
| `launch/gazebo.launch.py` | Sets `GZ_SIM_RESOURCE_PATH` so Gazebo finds the table model. Starts `robot_state_publisher` (sim time) and the **Gazebo server**, spawns the robot 2 m above the origin so it drops onto the table, opens the **Gazebo window** once the robot exists, and starts `ros_gz_bridge` for `/clock` and `/scan` from Gazebo to ROS. Arguments: `world_name`, `model`, and `gui` (default `true` when a display is available, otherwise headless). |
| `launch/display.launch.py` | Shows the robot model alone, without physics. Runs `robot_state_publisher`, `joint_state_publisher_gui` (sliders to turn the wheels) and RViz. Useful for checking URDF changes. |
| `worlds/empty_table.world` | Default world (SDF): ground plane, lighting, and a grid of `CoffeeTable` models forming one large table. |
| `worlds/empty.world` | A plain world with only a ground plane. |
| `models/aws_robomaker_residential_CoffeeTable_01/` | Coffee-table model (meshes, texture, `model.sdf`) from the [AWS RoboMaker small-house world](https://github.com/aws-robotics/aws-robomaker-small-house-world). The world file refers to it as `model://aws_robomaker_residential_CoffeeTable_01`. |
| `rviz/display.rviz` | Saved RViz layout: Grid, RobotModel, TF, LaserScan (red hit points with a 2-second trail) and LidarRays (green beams). Fixed frame is `odom`, so you see the robot drive across the grid. |
| `config/gui.config` | Gazebo window layout: the default panels with the camera starting close to the robot. |
| `package.xml` / `CMakeLists.txt` | Runtime dependencies (xacro, robot_state_publisher, RViz, ros_gz_sim, ros_gz_bridge, …) and installation of the `config`, `launch`, `models`, `urdf`, `rviz` and `worlds` folders. |

### `bot_controller`: wheel control

| File | Role |
|---|---|
| `config/bot_controllers.yaml` | `controller_manager` at 100 Hz with two controllers. **`wheel_controller`** (`diff_drive_controller`): wheel names, separation 0.55 m, radius 0.15 m, velocity and acceleration limits, publishes odometry and the `odom → base_footprint` TF. **`joint_state_broadcaster`**: publishes `/joint_states` so the wheels turn in RViz. |
| `launch/controller.launch.py` | Runs the `spawner` for each controller. They wait for the controller manager inside Gazebo, then load and activate the controllers. |
| `package.xml` / `CMakeLists.txt` | Runtime dependencies (`controller_manager`, `ros2_controllers`, `gz_ros2_control`, …) and installation of `config/` and `launch/`. |

### `bot_script`: Python nodes

| File | Role |
|---|---|
| `bot_script/edge_detection.py` | **The robot's behaviour.** Node `edge_avoidance_node` subscribes to `/scan` and publishes `geometry_msgs/TwistStamped` on `/wheel_controller/cmd_vel` at 10 Hz. See [the logic above](#1-how-the-application-works). Logs the "U" and "D" distances and "Edge ahead! Reversing and turning." |
| `bot_script/scan_rays.py` | **Visualization only.** Turns each `/scan` into green lines from the sensor to each hit point. Publishes them to RViz on `/scan_rays`, and also draws them in the Gazebo window through Gazebo's `/marker` service. It places them using the robot's world pose from Gazebo and the sensor offset from TF. |
| `setup.py` | Registers the two executables, `edge_detection` and `scan_rays`. |
| `setup.cfg` | Installs the executables where `ros2 run` / launch files expect them. |
| `resource/bot_script` | Empty marker file. ROS 2 needs it to find the package. |
| `package.xml` | Dependencies: rclpy, geometry_msgs, sensor_msgs, visualization_msgs, tf2_ros, and the Gazebo transport/message Python libraries. |

### ROS topics at a glance

| Topic | Type | From → To |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo lidar → bridge → `edge_detection`, `scan_rays`, RViz |
| `/wheel_controller/cmd_vel` | `geometry_msgs/TwistStamped` | `edge_detection` → `wheel_controller` |
| `/wheel_controller/odom` | `nav_msgs/Odometry` | `wheel_controller` → anyone |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` → `robot_state_publisher` |
| `/tf`, `/tf_static` | TF | `wheel_controller` (odom) and `robot_state_publisher` (robot links) |
| `/robot_description` | `std_msgs/String` | `robot_state_publisher` → Gazebo spawner, RViz |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → bridge → all nodes (`use_sim_time`) |
| `/scan_rays` | `visualization_msgs/Marker` | `scan_rays` → RViz |

---

## 4. Setting up VS Code and the Ubuntu/ROS container

ROS 2 Jazzy runs on Ubuntu 24.04. Rather than installing Ubuntu yourself, the project uses a **Dev Container**:
VS Code builds a Docker container with Ubuntu, ROS and all dependencies, and then works *inside* it. Your files
stay on your computer and are shared with the container.

### One-time setup on your computer

1. Install **Docker Desktop** (macOS/Windows) or **Docker Engine** (Linux) and make sure it is running.
   - On Windows, use the WSL 2 backend.
   - Give Docker at least 4 CPUs and 8 GB RAM (Docker Desktop → Settings → Resources). Gazebo needs it.
2. Install **Visual Studio Code**.
3. In VS Code, install the **Dev Containers** extension (`ms-vscode-remote.remote-containers`).

### Open the project in the container

1. In VS Code: **File → Open Folder…** and choose the `robot-project` folder.
2. VS Code shows *"Folder contains a Dev Container configuration file"*. Click **Reopen in Container**.
   (Or press `F1` → **Dev Containers: Reopen in Container**.)
3. The first time, VS Code builds the image from `.devcontainer/Dockerfile`. This downloads ROS, Gazebo and
   friends and takes a while (roughly 10–20 minutes). Later starts take seconds.
4. When it's done, the bottom-left corner of VS Code shows **"Dev Container: ROS 2 Jazzy - robot-project"**.
   Every terminal you open (**Terminal → New Terminal**) is now an Ubuntu shell with ROS 2 already sourced.
5. The virtual desktop starts automatically. VS Code may show a notification for port **6080**. Check the
   **Ports** tab in the bottom panel: port 6080 ("Desktop (noVNC)") should be listed.

> **Changed the Dockerfile or devcontainer.json?** Press `F1` → **Dev Containers: Rebuild Container** to apply it.

Check that ROS works in the container terminal:

```bash
echo $ROS_DISTRO        # → jazzy
ros2 --help             # ROS 2 command-line tool
gz sim --versions       # → 8.x (Gazebo Harmonic)
```

---

## 5. Building and running

All commands run in a VS Code terminal **inside the container**, from the workspace root.

```bash
cd /workspaces/robot-project

# (only needed if you add new dependencies to a package.xml)
rosdep install --from-paths src --ignore-src -r -y

# build all 4 packages
colcon build --symlink-install

# make this terminal aware of the built packages (do this in every new terminal)
source install/setup.bash

# start everything: Gazebo + robot + controllers + RViz + edge detection + ray drawing
ros2 launch bot_bringup simulated_robot.launch.py
```

`--symlink-install` links Python files and launch files instead of copying them, so you can edit those and
simply relaunch. **Rebuild** after adding a new file, a new executable in `setup.py`, or changing
`package.xml`/`CMakeLists.txt`.

Press **Ctrl+C** in the terminal to stop everything.

### Launch options

```bash
ros2 launch bot_bringup simulated_robot.launch.py world_name:=empty   # plain ground instead of the table
ros2 launch bot_bringup simulated_robot.launch.py gui:=false          # no Gazebo window (headless), faster
ros2 launch bot_description display.launch.py                         # robot model only, in RViz, with wheel sliders
```

---

## 6. Viewing the simulation

The container has no physical screen, so GUI programs open on a virtual desktop that you view in your browser:

**http://localhost:6080/vnc.html?autoconnect=true&resize=remote**

- `resize=remote` makes the desktop match your browser window exactly, which keeps everything sharp.
- Both **Gazebo** and **RViz** open maximized. Switch between them with the taskbar at the bottom.
- **In Gazebo:** the red robot drives on the table. The **green lines** coming from the green lidar box are the lidar beams.
- **In RViz:** the robot drives across the grid with its frames shown. **Green lines** are the beams and
  **red dots** are where they hit.
- If the desktop isn't running (for example after a container restart outside VS Code), run `./start_desktop.sh`.

Moving the camera:

| | Gazebo | RViz |
|---|---|---|
| Rotate | Middle-drag / Shift + left-drag | Left-drag |
| Pan | Left-drag | Middle-drag / Shift + left-drag |
| Zoom | Scroll / right-drag | Scroll / right-drag |
| Follow the robot | Right-click robot → *Follow* | — |

> Rendering runs on the CPU (no GPU in the container), so the simulation may run below real time.
> The percentage in Gazebo's bottom-right corner shows the real-time factor.

---

## 7. Useful commands

Run these in a second terminal (remember `source install/setup.bash`) while the simulation is running:

```bash
ros2 node list                                   # running nodes
ros2 topic list                                  # available topics
ros2 topic echo /scan                            # raw lidar data
ros2 topic echo /wheel_controller/odom --field pose.pose.position   # where the robot is
ros2 control list_controllers                    # should show both controllers "active"
ros2 run tf2_tools view_frames                   # writes frames.pdf showing the TF tree

# drive the robot yourself (stop the edge_detection node first, or it will override you)
ros2 topic pub -r 10 /wheel_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_footprint}, twist: {linear: {x: 0.2}, angular: {z: 0.5}}}"
```

---

## 8. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| RViz keeps printing **"Detected jump back in time"** | Two simulations are running at once, for example an old Gazebo left over from a crash. Stop everything, run `pkill -f "gz sim"`, then launch again. The taskbar should show only one Gazebo and one RViz. |
| `ros2 topic list` misses topics, or CLI commands see nothing | DDS discovery can be slow in a container. Run `ros2 daemon stop`, then try again. If processes were killed abruptly, `fastdds shm clean` removes stale shared-memory files. |
| **Gazebo window doesn't open** | `DISPLAY` must be `:1` (`echo $DISPLAY`) and the desktop must be running (`./start_desktop.sh`). Without a display, the launch automatically runs Gazebo headless. |
| Browser page at port 6080 doesn't load | Open VS Code's **Ports** tab and forward port `6080`, or run `./start_desktop.sh`. |
| Blurry desktop | Use the URL with `&resize=remote`. |
| `Package 'bot_...' not found` | You forgot `source install/setup.bash` in this terminal, or the build failed. |
| Controllers never become active | Gazebo didn't load the world or the robot. Read the first errors from `[gazebo-…]` in the launch output. |
| RViz error about `nav2_rviz_plugins/GoalTool` | Harmless. The saved layout includes a Nav2 toolbar button that isn't installed. |

---

## 9. Project history: what was done

**Original project (the author's work):**

- Designed the robot (`bot_description`): differential-drive base, castors, a front lidar tilted to look down
  at the table, Gazebo physics/friction tuning, and `ros2_control` hardware description.
- Built the table world from coffee-table models, plus a plain world.
- Configured the diff-drive and joint-state controllers (`bot_controller`).
- Wrote the edge-detection behaviour that reads two lidar beams and drives the robot (`bot_script`).
- Wrote the bringup launch file that ties it all together (`bot_bringup`).

**Changes made to get it running on ROS 2 Jazzy / Gazebo Harmonic, and visualization added:**

| Area | Change |
|---|---|
| Dependencies | Installed the missing packages (Gazebo `ros_gz`, `gz_ros2_control`, `ros2_control`, `ros2_controllers`, RViz) and listed them in the Dockerfile so a rebuilt container has them. |
| Gazebo plugin names | The old Ignition names (`ign_ros2_control`, `ignition-gazebo-sensors-system`) were replaced with the Jazzy/Harmonic names (`gz_ros2_control`, `gz-sim-sensors-system`). |
| Velocity commands | Jazzy's `diff_drive_controller` only accepts `TwistStamped` on `~/cmd_vel`, so `edge_detection.py` was updated and the removed `use_stamped_vel` parameter was dropped. |
| `bot_script` build | Added the required `resource/bot_script` marker file. |
| Missing table model | Added the AWS `CoffeeTable_01` model; the world file referenced it, but `models/` was empty. |
| `devcontainer.json` | Fixed invalid JSON. Added the virtual desktop, `DISPLAY`, and port forwarding. |
| Gazebo without a screen | The Gazebo window could not start in the container and took the simulator down with it. Added the browser desktop and a `gui` launch option that runs headless when there is no display. |
| Clean shutdown | Gazebo is now started directly (not through a wrapper shell), so Ctrl+C stops it fully instead of leaving it running in the background. Python nodes exit without tracebacks. |
| Lidar in RViz | Added `laser_frame` matching the sensor's tilt, so RViz draws the scan in the correct place. Larger scan points, and an `odom` fixed frame so the robot visibly moves. |
| Lidar beams | New `scan_rays` node draws the beams as green lines in both RViz and Gazebo. |
| Gazebo view | Camera starts close to the robot (`config/gui.config`). |
