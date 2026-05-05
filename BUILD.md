# Build & Test Guide

## Status (as of 2026-05-05)

| Package | Built | Notes |
|---|---|---|
| flexiv_rdk | ✅ colcon (wraps `rdk_install`) | one-time SDK install first |
| cynlr_arm_core | ✅ colcon | requires cmake overrides (spdlog/fastcdr/fastrtps) |
| cynlr_robot | ✅ colcon | hardware plugin + CynlrArmRegistry |
| cynlr_arm_node | ✅ colcon | cynlr_main executable + CynlrArmNode lib |
| cynlr_arm_controllers | ✅ colcon | 2 plugins: direct_cmd, cartesian |
| cynlr_arm_interfaces | ✅ colcon | — |
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

The Flexiv RDK v1.8 C++ library lives at `~/cynlr_software/flexiv_rdk/`.
Build and install it once:

```bash
cmake -S ~/cynlr_software/flexiv_rdk \
      -B ~/cynlr_software/flexiv_rdk/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=$HOME/cynlr_software/rdk_install
cmake --build ~/cynlr_software/flexiv_rdk/build --config Release -j$(nproc)
cmake --install ~/cynlr_software/flexiv_rdk/build
```

The install lands in `~/cynlr_software/rdk_install/`. It bundles:
- `libflexiv_rdk.a` — the RDK static library
- `libspdlog.a` (v1.14.1) — bundled, different version from system spdlog 1.12
- `libfastcdr.a`, `libfastrtps.a` — bundled FastDDS (different from ROS2's FastDDS)

> **Important**: `cynlr_arm_core` must be built with these bundled libraries (see Step 3)
> because `libflexiv_rdk.a` was compiled against them, not the system versions.

---

## Step 2 — Build `flexiv_rdk` colcon wrapper (once per clean workspace)

The `flexiv_rdk/` directory in this workspace is a thin colcon package that copies
the pre-built library from `rdk_install/` into the colcon install tree.

```bash
cd ~/cynlr_software/Cpp_App_Test
source /opt/ros/jazzy/setup.bash
colcon build --packages-select flexiv_rdk --cmake-args -DCMAKE_BUILD_TYPE=Release
```

This installs `install/flexiv_rdk/lib/libflexiv_rdk.a` and its cmake config files.
If `install/flexiv_rdk/` is missing (e.g. after a clean), this step must be re-run
before building `cynlr_arm_core`.

---

## Step 3 — Build `cynlr_arm_core` (requires cmake overrides)

`cynlr_arm_core` links against `libflexiv_rdk.a`, which was compiled with `rdk_install`'s
bundled spdlog (1.14.1), fastcdr, and fastrtps. These must be explicitly pointed to so
the linker resolves symbols correctly — the system spdlog (1.12.0) has an incompatible ABI.

```bash
cd ~/cynlr_software/Cpp_App_Test
source /opt/ros/jazzy/setup.bash && source install/setup.bash

colcon build --packages-select cynlr_arm_core \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -Dspdlog_DIR=$HOME/cynlr_software/rdk_install/lib/cmake/spdlog \
    -Dfastcdr_DIR=$HOME/cynlr_software/rdk_install/lib/cmake/fastcdr \
    -Dfastrtps_DIR=$HOME/cynlr_software/rdk_install/share/fastrtps/cmake
```

These overrides are cached in `build/cynlr_arm_core/CMakeCache.txt` after the first run,
so incremental rebuilds (`colcon build --packages-select cynlr_arm_core`) don't need them.

---

## Step 4 — Build remaining colcon packages

```bash
cd ~/cynlr_software/Cpp_App_Test
source /opt/ros/jazzy/setup.bash && source install/setup.bash

colcon build \
  --packages-select \
    cynlr_arm_interfaces \
    cynlr_robot \
    cynlr_arm_node \
    cynlr_arm_controllers \
    cynlr_arm_description \
    cynlr_moveit_config \
    flexiv_description \
    cynlr_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

---

## Full clean rebuild (all packages in order)

Use this sequence after a full `rm -rf build/ install/`:

```bash
cd ~/cynlr_software/Cpp_App_Test
source /opt/ros/jazzy/setup.bash

# 1. flexiv_rdk colcon wrapper
colcon build --packages-select flexiv_rdk --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 2. cynlr_arm_core with bundled library overrides
colcon build --packages-select cynlr_arm_core \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -Dspdlog_DIR=$HOME/cynlr_software/rdk_install/lib/cmake/spdlog \
    -Dfastcdr_DIR=$HOME/cynlr_software/rdk_install/lib/cmake/fastcdr \
    -Dfastrtps_DIR=$HOME/cynlr_software/rdk_install/share/fastrtps/cmake
source install/setup.bash

# 3. All remaining packages
colcon build \
  --packages-select \
    cynlr_arm_interfaces cynlr_robot cynlr_arm_node \
    cynlr_arm_controllers cynlr_arm_description \
    cynlr_moveit_config flexiv_description cynlr_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

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

ros2 topic hz /joint_states       # confirms state broadcaster is publishing
ros2 topic echo /arm_left_arm_state --once  # full arm state (pose, torques, fault flag)
```

---

## NRT motion actions (move_j, move_l, move_ptp)

The `CynlrArmNode` hosts three action servers per arm for NRT (non-real-time) motions.
These work while the JTC is active — the implementation pauses RT streaming, switches
the robot to NRT mode, executes the motion, then restores RT mode.

```bash
# Move to a joint-space pose (slow, safe velocities)
ros2 action send_goal /arm_left_move_j cynlr_arm_interfaces/action/MoveJ \
  "{target_positions: [0.0, -0.65, 0.0, 1.55, 0.0, 0.65, 0.0], max_joint_vel: 0.1, max_joint_acc: 0.05}"

# Move Cartesian (requires F/T sensor zeroing first — see below)
ros2 action send_goal /arm_left_move_l cynlr_arm_interfaces/action/MoveL \
  "{target_pose: [0.4, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0], max_linear_vel: 0.05, max_angular_vel: 0.0, max_linear_acc: 0.0}"
```

**Velocity guidelines** (WSL2 / non-RT host):
- Keep `max_joint_vel` ≤ 0.15 rad/s and `max_joint_acc` ≤ 0.1 rad/s² to stay within
  joint torque limits.
- For Cartesian moves, `max_linear_vel` ≤ 0.05 m/s is recommended for initial testing.

**F/T sensor zeroing** (required before `move_l` / Cartesian force control):
```bash
ros2 service call /arm_left_zero_ft_sensor cynlr_arm_interfaces/srv/Trigger '{}'
```

**Fault clearing** (if the robot enters fault state):
```bash
ros2 service call /arm_left_clear_fault cynlr_arm_interfaces/srv/Trigger '{}'
```
Note: this service calls `ClearFault()` on the Flexiv SDK which can block for a few
seconds on a MultiThreadedExecutor. If the arm_ops node becomes unresponsive, restart
the system.

---

## Escape hatch controller switching

```bash
# Switch from JTC to direct joint commands
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  '{deactivate_controllers: ["arm_left_jt_controller"], activate_controllers: ["arm_left_direct_cmd"], strictness: 2}'

ros2 topic pub /arm_left_joint_cmd_direct \
  cynlr_arm_interfaces/msg/JointCommand \
  "{mode: 0, joint_position: [0.0, -0.65, 0.0, 1.55, 0.0, 0.65, 0.0]}" --once

# Switch back to JTC
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  '{deactivate_controllers: ["arm_left_direct_cmd"], activate_controllers: ["arm_left_jt_controller"], strictness: 2}'
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
  arm_thread            — CynlrArmNode executor per arm (MultiThreadedExecutor)

  ControllerManager
    CynlrRobotInterface × N   (on_activate → registers CynlrArmHandle in CynlrArmRegistry)
    joint_state_broadcaster
    arm_*_jt_controller × N   (MoveIt / FollowJointTrajectory)
    arm_*_direct_cmd × N      (escape hatch — inactive by default)
    arm_*_cartesian × N       (escape hatch — inactive by default)

  CynlrArmNode × N
    Publishers: tcp_pose, arm_state
    Services:   clear_fault, set_tool, zero_ft_sensor
    Actions:    move_j, move_l, move_ptp  (NRT, blocking until motion complete)
```

---

## Key paths

| Item | Path |
|---|---|
| Flexiv RDK SDK install | `~/cynlr_software/rdk_install/` |
| System config (3-arm) | `cynlr_bringup/config/cynlr_system_config.yaml` |
| System config (1-arm) | `cynlr_bringup/config/cynlr_single_arm_config.yaml` |
| Hardware plugin | `cynlr_robot/src/cynlr_robot_interface.cpp` |
| Main executable | `cynlr_arm_node/src/cynlr_main.cpp` |
| Arm node | `cynlr_arm_node/src/cynlr_arm_node.cpp` |
| FlexivArm NRT impl | `cynlr_arm_core/src/flexiv/flexiv_arm.cpp` |

---

## Known issues / WSL2 notes

- **Timeliness warnings** (`Failure counter: N/3`): expected on WSL2. The non-RT scheduler
  causes occasional missed 1ms RT deadlines. The counter resets and doesn't cause motion
  failures in practice.
- **Overrun warnings** (`Total time > 2000us`): similarly expected on WSL2. Consider
  lowering `update_rate` to 200–250 Hz in `cynlr_system_config.yaml` if overruns
  are frequent.
- **NRT motion + JTC conflict**: after a `move_j`/`move_l` action completes, the JTC
  resumes and may briefly try to command the arm back to its pre-NRT hold position.
  Keeping NRT motion slow (vel ≤ 0.15 rad/s) avoids torque limit faults from this
  transition.
