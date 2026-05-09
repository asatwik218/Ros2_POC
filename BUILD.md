# Build & Test Guide

## Status (as of 2026-05-06)

| Package | Built | Notes |
|---|---|---|
| flexiv_rdk | ✅ cmake | pre-built static lib; `RDK_SUPPORT_ROS2_JAZZY=ON` required |
| cynlr_arm_core | ✅ cmake | pure C++; installed to `cynlr_install/` |
| cynlr_arm_interfaces | ✅ colcon | includes GetTool.srv, IsMotionRunning.srv |
| cynlr_robot | ✅ colcon | hardware plugin + CynlrArmRegistry |
| cynlr_arm_node | ✅ colcon | cynlr_main executable + CynlrArmNode lib |
| cynlr_arm_controllers | ✅ colcon | 2 plugins: direct_cmd, cartesian |
| cynlr_arm_description | ✅ colcon | xacro parses cleanly |
| flexiv_description | ✅ colcon | — |
| cynlr_moveit_config | ✅ colcon | — |
| cynlr_bringup | ✅ colcon | system launch verified |

**Deleted packages** (replaced by cynlr_robot + cynlr_arm_node):
- `cynlr_arm_service` — removed
- `cynlr_hardware` — removed (was `CynlrHardwareInterface`)
- `cynlr_arm_controllers/cynlr_state_broadcaster` — removed
- `cynlr_arm_controllers/cynlr_arm_services` — removed
- `cynlr_arm_controllers/cynlr_nrt_passthrough_controller` — removed

---

## One-time prerequisites

```bash
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-realtime-tools \
  ros-jazzy-hardware-interface \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-controller-manager-msgs
```

---

## Step 1 — Build flexiv_rdk SDK (one-time, installs to `rdk_install/`)

The Flexiv RDK v1.8 submodule lives at `flexiv_rdk/` inside this workspace.
`-DRDK_SUPPORT_ROS2_JAZZY=ON` is **required** — it downloads the `.a` variant that was
compiled against fastcdr 2.2.7 (matching ROS2 Jazzy). Without it the link fails with
undefined `serializeArray` symbols.

```bash
cd ~/cynlr_software/Cpp_App_Test
git submodule update --init --recursive

cmake -S flexiv_rdk -B flexiv_rdk/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=$HOME/cynlr_software/rdk_install \
      -DRDK_SUPPORT_ROS2_JAZZY=ON
cmake --build flexiv_rdk/build -j$(nproc)
cmake --install flexiv_rdk/build
```

The install lands in `~/cynlr_software/rdk_install/`.

---

## Step 2 — Build `cynlr_arm_core` (standalone cmake, installs to `cynlr_install/`)

`cynlr_arm_core` is a pure CMake package — not a colcon package. Build and install it
before running colcon. The colcon build picks it up via `CMAKE_PREFIX_PATH`.

```bash
cd ~/cynlr_software/Cpp_App_Test

cmake -S cynlr_arm_core -B cynlr_arm_core/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="$HOME/cynlr_software/rdk_install" \
      -DCMAKE_INSTALL_PREFIX=$HOME/cynlr_software/cynlr_install
cmake --build cynlr_arm_core/build -j$(nproc)
cmake --install cynlr_arm_core/build

# Install into the colcon workspace install tree as well (so downstream packages find it)
cmake --install cynlr_arm_core/build \
      --prefix ~/cynlr_software/Cpp_App_Test/install/cynlr_arm_core
```

Run unit tests (no ROS required):
```bash
ctest --test-dir cynlr_arm_core/build --output-on-failure
```

---

## Step 3 — Build all colcon packages

```bash
cd ~/cynlr_software/Cpp_App_Test
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select \
    cynlr_arm_interfaces \
    cynlr_robot \
    cynlr_arm_node \
    cynlr_arm_controllers \
    cynlr_arm_description \
    flexiv_description \
    cynlr_moveit_config \
    cynlr_bringup \
  --cmake-args \
    -DCMAKE_PREFIX_PATH="$HOME/cynlr_software/cynlr_install;/opt/ros/jazzy"

source install/setup.bash
```

---

## Full clean rebuild (from zero)

Use this after wiping `build/`, `install/`, `rdk_install/`, and `cynlr_install/`.

```bash
cd ~/cynlr_software/Cpp_App_Test
rm -rf build install log cynlr_arm_core/build
rm -rf $HOME/cynlr_software/rdk_install $HOME/cynlr_software/cynlr_install
git submodule update --init --recursive

# 1. flexiv_rdk (downloads ROS2-Jazzy-compatible .a from GitHub releases)
cmake -S flexiv_rdk -B flexiv_rdk/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=$HOME/cynlr_software/rdk_install \
      -DRDK_SUPPORT_ROS2_JAZZY=ON
cmake --build flexiv_rdk/build -j$(nproc)
cmake --install flexiv_rdk/build

# 2. cynlr_arm_core (pure cmake — must install before colcon)
cmake -S cynlr_arm_core -B cynlr_arm_core/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="$HOME/cynlr_software/rdk_install" \
      -DCMAKE_INSTALL_PREFIX=$HOME/cynlr_software/cynlr_install
cmake --build cynlr_arm_core/build -j$(nproc)
cmake --install cynlr_arm_core/build
cmake --install cynlr_arm_core/build \
      --prefix ~/cynlr_software/Cpp_App_Test/install/cynlr_arm_core
ctest --test-dir cynlr_arm_core/build --output-on-failure

# 3. All ROS packages
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-select \
    cynlr_arm_interfaces cynlr_robot cynlr_arm_node \
    cynlr_arm_controllers cynlr_arm_description \
    flexiv_description cynlr_moveit_config cynlr_bringup \
  --cmake-args \
    -DCMAKE_PREFIX_PATH="$HOME/cynlr_software/cynlr_install;/opt/ros/jazzy"

source install/setup.bash
```

---

## Launch — single arm

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 launch cynlr_bringup cynlr_system.launch.py \
  use_rviz:=false use_moveit:=false \
  config_file:=$(ros2 pkg prefix cynlr_bringup)/share/cynlr_bringup/config/cynlr_single_arm_config.yaml
```

In a second terminal:
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 control list_controllers
# Expected:
#   joint_state_broadcaster    [active]
#   arm_left_jt_controller     [active]
#   arm_left_direct_cmd        [inactive]
#   arm_left_cartesian         [inactive]

ros2 topic hz /joint_states
ros2 topic echo /arm_left_arm_state --once   # includes motion_running field
```

---

## ROS API reference (prefix = `arm_left_` for left arm)

### Topics published at 100 Hz

| Topic | Type |
|---|---|
| `arm_left_arm_state` | `cynlr_arm_interfaces/msg/ArmState` |
| `arm_left_tcp_pose` | `geometry_msgs/msg/PoseStamped` |
| `arm_left_ft_raw` | `geometry_msgs/msg/WrenchStamped` |
| `arm_left_ext_wrench_tcp` | `geometry_msgs/msg/WrenchStamped` |
| `arm_left_ext_wrench_world` | `geometry_msgs/msg/WrenchStamped` |

`ArmState` includes a `motion_running` bool (true while a `move_j/l/ptp` action is executing).

### Lifecycle services

```bash
ros2 service call /arm_left_connect          cynlr_arm_interfaces/srv/Trigger '{}'
ros2 service call /arm_left_disconnect       cynlr_arm_interfaces/srv/Trigger '{}'
ros2 service call /arm_left_enable           cynlr_arm_interfaces/srv/Trigger '{}'
ros2 service call /arm_left_stop             cynlr_arm_interfaces/srv/Trigger '{}'  # abort NRT motion
ros2 service call /arm_left_clear_fault      cynlr_arm_interfaces/srv/Trigger '{}'
ros2 service call /arm_left_zero_ft_sensor   cynlr_arm_interfaces/srv/Trigger '{}'
ros2 service call /arm_left_is_motion_running cynlr_arm_interfaces/srv/IsMotionRunning '{}'
```

### Tool services

```bash
ros2 service call /arm_left_get_tool    cynlr_arm_interfaces/srv/GetTool '{}'
ros2 service call /arm_left_set_tool    cynlr_arm_interfaces/srv/SetTool \
  '{mass_kg: 0.5, com: [0,0,0.05], inertia: [0,0,0,0,0,0], tcp_pose: [0,0,0.1,1,0,0,0]}'
ros2 service call /arm_left_update_tool cynlr_arm_interfaces/srv/SetTool \
  '{mass_kg: 0.8, com: [0,0,0.05], inertia: [0,0,0,0,0,0], tcp_pose: [0,0,0.1,1,0,0,0]}'
```

### NRT motion actions

The three action servers issue discrete NRT moves. They are **non-blocking** internally —
`CynlrArmNode` polls `is_motion_complete()` and publishes feedback until done.
Cancel a goal with `ros2 action cancel` and the arm stops immediately.

```bash
# Joint-space move
ros2 action send_goal /arm_left_move_j cynlr_arm_interfaces/action/MoveJ \
  '{target_positions: [0.0,-0.65,0.0,1.55,0.0,0.65,0.0], max_joint_vel: 0.1, max_joint_acc: 0.05}' \
  --feedback

# Cartesian linear move (zero ft_sensor first)
ros2 service call /arm_left_zero_ft_sensor cynlr_arm_interfaces/srv/Trigger '{}'
ros2 action send_goal /arm_left_move_l cynlr_arm_interfaces/action/MoveL \
  '{target_pose: [0.4,0.0,0.5,1.0,0.0,0.0,0.0], max_linear_vel: 0.05}' \
  --feedback

# PTP (point-to-point Cartesian)
ros2 action send_goal /arm_left_move_ptp cynlr_arm_interfaces/action/MovePTP \
  '{target_pose: [0.4,0.0,0.5,1.0,0.0,0.0,0.0], max_joint_vel: 0.1}' \
  --feedback
```

**Velocity guidelines** (WSL2 / non-RT host):
- `max_joint_vel` ≤ 0.15 rad/s, `max_joint_acc` ≤ 0.1 rad/s²
- `max_linear_vel` ≤ 0.05 m/s for initial Cartesian testing

### Controller switching

Controller switches are **rejected while an NRT motion is running**. Call
`/arm_left_stop` (or cancel the action) first, then switch.

```bash
# Switch from JTC to direct joint commands
ros2 control switch_controllers \
  --deactivate arm_left_jt_controller \
  --activate   arm_left_direct_cmd \
  --strict

ros2 topic pub /arm_left_joint_cmd_direct \
  cynlr_arm_interfaces/msg/JointCommand \
  '{mode: 0, joint_position: [0.0,-0.65,0.0,1.55,0.0,0.65,0.0]}' --once

# Switch to Cartesian streaming
ros2 control switch_controllers \
  --deactivate arm_left_direct_cmd \
  --activate   arm_left_cartesian \
  --strict
```

---

## Architecture overview

```
cynlr_system_config.yaml  (arm count, names, serials, mounts, vendor)
         │
         ▼
cynlr_system.launch.py
  generates URDF + controller YAML → spawns cynlr_main

cynlr_main  (3 threads)
  cm_executor         — ROS callbacks for ControllerManager
  cm_update_thread    — 500 Hz: read() → update() → write()
  arm_thread          — CynlrArmNode executor per arm

  ControllerManager
    CynlrRobotInterface × N   (registers CynlrArmHandle in CynlrArmRegistry on activate)
    joint_state_broadcaster
    arm_*_jt_controller × N   (MoveIt / FollowJointTrajectory)
    arm_*_direct_cmd    × N   (inactive by default)
    arm_*_cartesian     × N   (inactive by default)

  CynlrArmNode × N
    Publishers:  arm_state, tcp_pose, ft_raw, ext_wrench_*  @ 100 Hz
    Services:    connect, disconnect, enable, stop, clear_fault,
                 zero_ft_sensor, set_tool, update_tool, get_tool, is_motion_running
    Actions:     move_j, move_l, move_ptp  (async NRT with feedback + cancel)
```

---

## Key paths

| Item | Path |
|---|---|
| Flexiv RDK SDK install | `~/cynlr_software/rdk_install/` |
| cynlr_arm_core install | `~/cynlr_software/cynlr_install/` |
| System config (3-arm) | `cynlr_bringup/config/cynlr_system_config.yaml` |
| System config (1-arm) | `cynlr_bringup/config/cynlr_single_arm_config.yaml` |
| Hardware plugin | `cynlr_robot/src/cynlr_robot_interface.cpp` |
| Main executable | `cynlr_arm_node/src/cynlr_main.cpp` |
| Arm node | `cynlr_arm_node/src/cynlr_arm_node.cpp` |
| FlexivArm NRT impl | `cynlr_arm_core/src/flexiv/flexiv_arm.cpp` |

---

## Known issues / WSL2 notes

- **Timeliness warnings** (`Failure counter: N/3`): expected on WSL2. The non-RT scheduler
  causes occasional missed 1ms RT deadlines. Does not cause motion failures in practice.
- **Overrun warnings** (`Total time > 2000us`): also expected on WSL2. Lower `update_rate`
  to 200–250 Hz in `cynlr_system_config.yaml` if overruns are frequent.
- **NRT motion + JTC transition**: after a `move_j`/`move_l`/`move_ptp` action completes,
  the JTC resumes at the current joint position (cmd_pos_ is reseeded from hw_pos_ on the
  NRT→RT edge). Keep NRT motion slow (vel ≤ 0.15 rad/s) to avoid large position steps.
- **Controller switch interlock**: `prepare_command_mode_switch` rejects switches while
  `nrt_active() == true`. Call `/arm_left_stop` first, confirm `is_motion_running` returns
  false, then switch.
