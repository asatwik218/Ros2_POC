# Build & Test Guide

## Status (as of 2026-04-29)

| Package | Built | Notes |
|---|---|---|
| cynlr_arm_core | ✅ (Conan, installs to `~/cynlr_software/cynlr_install`) | unit tests pass |
| flexiv_description | ✅ colcon | humble branch |
| cynlr_arm_interfaces | ✅ colcon | — |
| cynlr_robot | ✅ colcon | hardware plugin + CynlrArmRegistry |
| cynlr_arm_node | ✅ colcon | cynlr_main executable + CynlrArmNode lib |
| cynlr_arm_controllers | ✅ colcon | 2 plugins: direct_cmd, cartesian |
| cynlr_arm_description | ✅ colcon | xacro parses cleanly |
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
  ros-jazzy-joint-trajectory-controller
```

---

## Build flexiv_rdk from submodule (one-time)

`flexiv_rdk` (v1.8) is included as a git submodule at `flexiv_rdk/`.

```bash
cd ~/cynlr_software/Cpp_App_Test
git submodule update --init --recursive

# Build and install flexiv_rdk
cmake -S flexiv_rdk -B flexiv_rdk/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/cynlr_software/rdk_install
cmake --build flexiv_rdk/build --config Release -j$(nproc)
cmake --install flexiv_rdk/build
```

The install lands in `~/cynlr_software/rdk_install/` (already referenced by `cynlr_arm_core`).

---

## Build cynlr_arm_core (one-time)

`cynlr_arm_core` is a pure-CMake package (not colcon). It wraps flexiv_rdk behind
a stable facade and installs to `cynlr_install/`.

```bash
cd ~/cynlr_software/Cpp_App_Test/cynlr_arm_core
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$HOME/cynlr_software/rdk_install" \
  -DCMAKE_INSTALL_PREFIX=$HOME/cynlr_software/cynlr_install
cmake --build build --config Release -j$(nproc)
cmake --install build
```

---

## Build all colcon packages

```bash
cd ~/cynlr_software/Cpp_App_Test
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select \
    flexiv_description \
    cynlr_arm_interfaces \
    cynlr_robot \
    cynlr_arm_node \
    cynlr_arm_controllers \
    cynlr_arm_description \
    cynlr_moveit_config \
    cynlr_bringup \
  --cmake-args \
    -DCMAKE_PREFIX_PATH="$HOME/cynlr_software/cynlr_install;/opt/ros/jazzy"

source install/setup.bash
```

---

## System smoke test — single arm (recommended first run)

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
# Expected: 4 controllers
#   joint_state_broadcaster    [active]
#   arm_left_jt_controller     [active]
#   arm_left_direct_cmd        [inactive]
#   arm_left_cartesian         [inactive]

ros2 topic hz /joint_states   # confirms state broadcaster is publishing
```

## System smoke test — three arms

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch cynlr_bringup cynlr_system.launch.py use_rviz:=false use_moveit:=false
# Default config: cynlr_system_config.yaml (arm_left=Flexiv, arm_center/right=sim)
```

Expected: 10 controllers total (4×arm_left + 3×arm_center + 3×arm_right).
On WSL: arm_center/arm_right JT controllers may timeout due to non-RT jitter from
Flexiv's stream_command blocking the update thread. On a real-time OS this is fixed.

---

## Escape hatch switching

With system running, test controller switches:

```bash
# Hatch #1 — direct joint commands (arm holds RT streaming; JT releases it)
ros2 control switch_controllers \
  --deactivate arm_left_jt_controller --activate arm_left_direct_cmd --strict
ros2 topic pub /arm_left_joint_cmd_direct \
  cynlr_arm_interfaces/msg/JointCommand \
  "{mode: 0, joint_position: [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0]}" --once

# Switch back
ros2 control switch_controllers \
  --deactivate arm_left_direct_cmd --activate arm_left_jt_controller --strict

# Hatch #2 — Cartesian command
ros2 control switch_controllers \
  --deactivate arm_left_jt_controller --activate arm_left_cartesian --strict
ros2 topic pub /arm_left_cartesian_cmd \
  cynlr_arm_interfaces/msg/CartesianCommand \
  "{pose: [0.4, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0], wrench: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --once

# Switch back
ros2 control switch_controllers \
  --deactivate arm_left_cartesian --activate arm_left_jt_controller --strict
```

---

## Architecture overview

```
cynlr_system_config.yaml  (single source of truth: arm count, names, serials, mounts)
         │
         ▼
cynlr_system.launch.py
  generates URDF string + controller YAML tempfile → spawns cynlr_main

cynlr_main  (3 threads)
  cm_executor (main)    — ROS service/topic callbacks for ControllerManager
  cm_update_thread      — 500 Hz: cm->read(), cm->update(), cm->write()
  arm_thread            — CynlrArmNode executor per arm

  ControllerManager
    CynlrRobotInterface × N   (on_activate → registers CynlrArmHandle in CynlrArmRegistry)
    joint_state_broadcaster
    arm_*_jt_controller × N   (MoveIt / FollowJointTrajectory)
    arm_*_direct_cmd × N      (escape hatch — inactive by default)
    arm_*_cartesian × N       (escape hatch — inactive by default)

  CynlrArmNode × N
    Publishers: tcp_pose, wrench_tcp, wrench_raw, arm_state
    Services:   clear_fault, set_tool, zero_ft_sensor
    Actions:    move_l, move_j, move_ptp
```

---

## Key paths

| Item | Path |
|---|---|
| cynlr_arm_core install | `~/cynlr_software/cynlr_install/` |
| flexiv_description | `Cpp_App_Test/flexiv_description/` (humble branch) |
| System config (3-arm) | `cynlr_bringup/config/cynlr_system_config.yaml` |
| System config (1-arm) | `cynlr_bringup/config/cynlr_single_arm_config.yaml` |
| Hardware plugin | `cynlr_robot/src/cynlr_robot_interface.cpp` |
| Main executable | `cynlr_arm_node/src/cynlr_main.cpp` |
| Arm node | `cynlr_arm_node/src/cynlr_arm_node.cpp` |
