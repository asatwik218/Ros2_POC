# Robot Arm Service Implementation Plan

## Overview

Implement a flexible ROS2 architecture for the CynLR physical intelligence device: three Flexiv Rizon 7-DOF arms + stereo cameras. Supports ros2_control, MoveIt, Gazebo, and custom motion planning/IK — all swappable without rewriting the hardware layer. `cynlr_arm_core` stays untouched (pure C++20, no ROS).

**Architecture pivot (2026-04-23):** The original plan targeted a single-arm standalone service node. The architecture was expanded to a full multi-arm ros2_control system after recognising that CynLR is a physical intelligence device, not an industrial fixed-task robot, and requires maximum flexibility in motion planning and control.

---

## Package Structure

```
Cpp_App_Test/
├── cynlr_arm_core/          UNCHANGED — pure C++20, vendor-agnostic
├── cynlr_arm_interfaces/    EXTENDED — added SwitchController.srv
├── cynlr_arm_service/       SLIMMED  — supervision only (fault, lifecycle, tool, GPIO)
├── cynlr_hardware/          NEW — ros2_control SystemInterface plugin
├── cynlr_arm_description/   NEW — URDF/xacro for 3-arm system
├── cynlr_arm_controllers/   NEW — custom ros2_control controller plugins
│   ├── cynlr_state_broadcaster/
│   ├── cynlr_direct_command_controller/   (escape hatch #1)
│   ├── cynlr_nrt_passthrough_controller/  (escape hatch #2)
│   └── cynlr_cartesian_controller/        (escape hatch #3)
├── cynlr_moveit_config/     PENDING — MoveIt SRDF/kinematics for 3 arms
├── cynlr_bringup/           NEW — launch files + controller YAML
└── cynlr_camera/            PENDING — stereo camera lifecycle nodes
```

---

## Current State Analysis

No implementation exists yet for new packages. This is greenfield for ros2_control layer.

**Established patterns to follow:**
- Conan package: `cynlr_modbus/conanfile.py` — use identical structure
- CMakeLists: `cynlr_modbus/CMakeLists.txt` — C++20, BUILD_TESTING, GTest pattern
- `tl::expected` already in Conan cache (used by cynlr_modbus)
- `flexiv_rdk/1.8` used — v1.8 and v1.9 share identical public API
- `spdlog` available via Conan

**Development platform: WSL2 (ubuntu-wsl = Ubuntu 24.04, ROS2 Jazzy)**
- ROS2 for Windows (Jazzy) does not support Windows 11 — WSL2 is the permanent solution
- cynlr_arm_core built as SHARED on Linux with `-Wl,--exclude-libs,ALL -Wl,-Bsymbolic` to hide Flexiv RDK's FastDDS symbols from colliding with ROS2's FastDDS
- RPATH pattern: `get_target_property(_loc cynlr::cynlr_arm_core IMPORTED_LOCATION_RELEASE)` → embed libdir in INSTALL_RPATH

---

## Desired End State

1. `ros2 launch cynlr_bringup cynlr_system.launch.py vendor:=sim` — all three arms come up, state broadcasters publish, joint_state_broadcaster publishes, RViz shows robot
2. MoveIt plans and executes a trajectory through `arm_left_jt_controller`
3. Escape hatch switching: `ros2 control switch_controllers --activate arm_left_direct_cmd --deactivate arm_left_jt_controller` — publish `JointCommand` → arm holds position
4. `ros2 control switch_controllers --activate arm_left_nrt` → send MoveL action goal → arm moves via SimArm NRT planner
5. Real hardware: replace `vendor:=sim` with `vendor:=flexiv` plus serial numbers → identical test sequence on real robots

---

## Architecture: Three Escape Hatches

All escape hatches coexist in the same controller manager. Only one can own a given arm's command interfaces at a time. Switch at runtime via `ros2 control switch_controllers`.

| Hatch | Controller | Use case |
|---|---|---|
| Default | `arm_*_jt_controller` (JTC) | MoveIt / FollowJointTrajectory |
| #1 | `arm_*_direct_cmd` | Learning policies, custom joint-space planners |
| #2 | `arm_*_nrt` | High-level task sequencing via arm's NRT planner |
| #3 | `arm_*_cartesian` | Force control, visual servoing, Cartesian policies |

---

## What We're NOT Doing

- Orchestrator / BT engine
- IK solvers (separate library, MoveIt custom IK plugin is optional bonus)
- C ABI DLL layer for LabVIEW

---

## Phase 1: cynlr_arm_core — Types, Error, and Interfaces ✅ DONE

Scaffold the core library. Define all types, error codes, ArmConfig, ArmInterface abstract base, and capability interfaces.

---

## Phase 2: cynlr_arm_core — SimArm Implementation ✅ DONE

`SimArm` — fully functional simulated arm. Implements `ArmInterface`, `ForceControllable`, `NullSpaceConfigurable`. Joint integrator + FK for Cartesian state.

---

## Phase 3: cynlr_arm_core — FlexivArm Implementation ✅ DONE

`FlexivArm` wrapping `flexiv::rdk::Robot`. All capability interfaces. RT streaming methods compiled only on Linux via `#ifdef __linux__`.

**Additional cross-platform work:**
- `conanfile.py`: forces `spdlog.shared=True` + `fmt.shared=True` on Windows; OS-conditional flexiv_rdk component name
- `cmake/CopyRuntimeDlls.cmake`: post-build DLL copy (Windows only)
- `flexiv_arm.cpp`: v1.8 API, arrays not vectors, 4-arg SendJointPosition
- 19/19 unit tests pass on Windows MSVC

---

## Phase 4: cynlr_arm_interfaces — ROS2 Message Package ✅ DONE

ROS2 package with all message, service, and action definitions:
- `msg/`: ArmState, JointCommand, CartesianCommand, FaultEvent, OperationalStatus
- `srv/`: Connect, SetMode, SetTool, Trigger, SetDigitalOutputs, GetDigitalInputs
- `action/`: MoveL, MoveJ, MovePTP

---

## Phase 5: cynlr_arm_service — Mode Manager, RT Thread Scaffold ✅ DONE

`ModeManager` state machine and `RTThread` with `TripleBuffer<T>` lock-free buffers. Node skeleton with lifecycle callbacks.

---

## Phase 6: cynlr_arm_service — NRT Services and Actions ✅ DONE

All NRT services (connect, enable, stop, clear_fault, set_tool, etc.) and actions (MoveL, MoveJ, MovePTP). Services enforce mode via ModeManager. Actions poll `is_motion_complete()` at 50Hz and publish feedback.

---

## Phase 7: cynlr_arm_service — RT Path (Command Subscribe + State Publish) ✅ DONE

RT topic subscribers write to `RTThread` lock-free buffer. State publish timer reads from RT thread state buffer. SetMode service starts/stops RT thread. Fault monitoring timer at 5Hz.

---

## Phase 8: Integration Tests 🔲 PENDING

Automated integration tests using `launch_testing`. Launch arm service with SimArm, drive via ROS2 client calls, verify behavior.

**Build order (WSL2):**
```bash
# Package cynlr_arm_core
cd /path/to/cynlr_arm_core
conan create . --profile=linux-gcc -s build_type=Release --build=missing

# Install CMake config for colcon to find
conan install --requires=cynlr_arm_core/0.1.0 --profile=linux-gcc -s build_type=Release \
    -g CMakeDeps --output-folder=/opt/cynlr_cmake

# Build ROS2 packages
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cynlr_arm_interfaces cynlr_arm_service \
    --cmake-args -DCMAKE_PREFIX_PATH=/opt/cynlr_cmake
```

---

## Architecture Pivot (2026-04-23): ros2_control Multi-Arm System

At this point, the architecture expanded from a standalone single-arm service to a full ros2_control-based multi-arm system. The `cynlr_arm_service` was slimmed to a supervision-only node; motion control responsibilities moved into the ros2_control layer.

**Key design decisions:**
- Three separate hardware plugin instances (one per arm prefix) in a single controller manager at 1000 Hz
- `ArmState*` and `ArmInterface*` pointers passed through `double*` StateInterface slots using the bit_cast trick from Flexiv's `flexiv_robot_states.hpp`
- NaN sentinel in all command buffers — `write()` skips sending if any command is NaN
- `vendor:=sim` → `SimArm`, `vendor:=flexiv` → real hardware, zero code change

---

## Phase 9: cynlr_hardware — ros2_control Plugin ✅ DONE

**Files:**
- `cynlr_hardware/include/cynlr_hardware/cynlr_hardware_interface.hpp`
- `cynlr_hardware/src/cynlr_hardware_interface.cpp`
- `cynlr_hardware/cynlr_hardware_plugin.xml`
- `cynlr_hardware/CMakeLists.txt`, `package.xml`

**Class: `CynlrHardwareInterface : hardware_interface::SystemInterface`**

`on_init()`: reads `prefix`, `vendor`, `serial_number`, `ip_address` from URDF `<hardware>` params; calls `create_arm(config)`; NaN-fills all command buffers.

`on_activate()`: `arm_->connect()` → `arm_->clear_fault()` → `arm_->enable()` → seeds `cmd_pos_` from current state.

`on_deactivate()`: `arm_->stop_streaming()` → `arm_->stop()` → `arm_->disconnect()`.

`export_state_interfaces()`:
- 7 joints × {position, velocity, effort}
- FT sensor: 18 doubles (`{prefix}ft_sensor/raw_0..5`, `ext_tcp_0..5`, `ext_world_0..5`)
- TCP: 7 pose + 6 velocity doubles
- `{prefix}cynlr_arm_state/full_state_ptr` — `ArmState*` bit-cast as `double*`
- `{prefix}cynlr_arm_ctrl/arm_interface_ptr` — `ArmInterface*` bit-cast as `double*`
- GPIO: 18 input doubles

`export_command_interfaces()`:
- 7 joints × {position, velocity, effort}
- Cartesian: 13 GPIO doubles (`{prefix}cartesian_cmd/pose_0..6`, `wrench_0..5`)
- GPIO: 18 output doubles

`write()` dispatches by `ActiveMode`: POSITION → `JOINT_POSITION` stream, EFFORT → `JOINT_TORQUE` stream, CARTESIAN → `CARTESIAN_MOTION_FORCE` stream.

**CMakeLists:** links `cynlr::cynlr_arm_core`, embeds RPATH to libdir, `pluginlib_export_plugin_description_file`.

---

## Phase 10: cynlr_arm_description — Three-Arm URDF ✅ DONE

**Files:**
- `cynlr_arm_description/urdf/cynlr_rizon7_macro.urdf.xacro` — per-arm macro with full ros2_control block
- `cynlr_arm_description/urdf/cynlr_arm_system.urdf.xacro` — top-level, instantiates 3 arms

**xacro args:** `sn_left`, `sn_center`, `sn_right`, `vendor` (default `sim`)

Each arm: `<ros2_control name="${prefix}system" type="system">` block with all interface declarations. Three separate hardware plugin instances. Joint names: `arm_left_joint1` … `arm_right_joint7`.

**Note:** Depends on `flexiv_description` (external: https://github.com/flexivrobotics/flexiv_description, branch humble). Must be cloned separately before building.

---

## Phase 11: cynlr_arm_controllers — Four Controller Plugins ✅ DONE

All four built as SHARED libs in a single `cynlr_arm_controllers` package.

### CynlrStateBroadcaster
Claims `{prefix}cynlr_arm_state/full_state_ptr` state interface. Recovers `ArmState*` via bit_cast. Publishes `/{prefix}arm_state` (ArmState), TCP pose, FT wrenches using `RealtimePublisher`. Configurable decimation for full ArmState vs. high-freq TCP/wrench.

### CynlrDirectCommandController (Escape Hatch #1)
Claims 7 joint position + velocity command interfaces. Subscribes to `/{prefix}joint_cmd_direct` (JointCommand) with BEST_EFFORT QoS. Uses `RealtimeBuffer` for NRT→RT handoff. `on_activate()` seeds from current positions to prevent lurch.

### CynlrNrtPassthroughController (Escape Hatch #2)
Claims `{prefix}cynlr_arm_ctrl/arm_interface_ptr` state interface (read-only). `on_activate()` recovers `ArmInterface*` via bit_cast. Hosts MoveL, MoveJ, MovePTP action servers. Goals execute in detached threads calling `arm_->move_l/j/ptp()` directly.

### CynlrCartesianController (Escape Hatch #3)
Claims 13 `cartesian_cmd` GPIO command interfaces (7 pose + 6 wrench). Subscribes to `/{prefix}cartesian_cmd` (CartesianCommand) with BEST_EFFORT QoS. NaN-fills on deactivate. Hardware `write()` detects CARTESIAN mode and calls `arm_->stream_command({CARTESIAN_MOTION_FORCE, ...})`.

---

## Phase 12: cynlr_arm_interfaces — Extended ✅ DONE

Added `SwitchController.srv` for programmatic per-arm controller switching:

```
string arm           # "left", "center", "right"
string[] activate
string[] deactivate
uint8 strictness     # 0=BEST_EFFORT, 1=STRICT
---
bool success
string message
```

Updated `CMakeLists.txt` to register the new service.

**Updated OperationalStatus.msg comment:** `mode` field now maps to `ArmMode` enum values: `0=IDLE 1=CONNECTED 2=ENABLED 3=FAULT` (NRT_MODE and RT_MODE removed — ros2_control manages these now).

---

## Phase 13: cynlr_bringup — Launch Files + Controller YAML ✅ DONE

**Files:**
- `cynlr_bringup/config/cynlr_controllers.yaml` — controller manager at 1000 Hz, all 12 controllers (3 arms × 4 variants) with full per-controller parameters
- `cynlr_bringup/launch/cynlr_system.launch.py`

**Launch args:** `sn_left`, `sn_center`, `sn_right`, `vendor` (default `sim`), `use_moveit`, `use_rviz`

**Startup chain (OnProcessExit):**
1. `ros2_control_node` + `robot_state_publisher`
2. `joint_state_broadcaster`
3. `arm_left_state_broadcaster` → `arm_center_state_broadcaster` → `arm_right_state_broadcaster`
4. `arm_left_jt_controller` → `arm_center_jt_controller` → `arm_right_jt_controller`
5. All 9 escape-hatch controllers loaded `--inactive` (direct_cmd, nrt, cartesian per arm)
6. `rviz2` (if `use_rviz:=true`) + `move_group` (if `use_moveit:=true`)

**Escape hatch switching:**
```bash
ros2 control switch_controllers \
  --activate arm_left_direct_cmd \
  --deactivate arm_left_jt_controller arm_left_nrt arm_left_cartesian
```

---

## Phase 14: cynlr_arm_service — Slimmed to Supervision Node ✅ DONE

**Removed** (moved to ros2_control layer):
- Action servers (MoveL, MoveJ, MovePTP) → `CynlrNrtPassthroughController`
- `sub_joint_cmd_`, `sub_cart_cmd_` → handled by `CynlrDirectCommandController` / `CynlrCartesianController`
- `state_publish_timer_` (ArmState) → `CynlrStateBroadcaster` handles this
- `RTThread` → controller_manager is the 1kHz loop now
- `SetMode` service → `ros2 control switch_controllers` handles mode switching
- `NRT_MODE` and `RT_MODE` from `ArmMode` enum

**Kept** (supervision and NRT configuration only):
- All NRT config services: connect, disconnect, enable, stop, clear_fault, set_tool, zero_ft, auto_recovery, set/get digital I/O
- `pub_fault_`, `pub_status_`
- `fault_poll_timer_` at 5Hz (200ms)
- `ModeManager` with 4 states: IDLE → CONNECTED → ENABLED → FAULT

**Role:** Supervision node. Holds a separate NRT `ArmInterface` instance for configuration commands. Flexiv SDK supports concurrent connections for NRT operations alongside the 1kHz RT connection owned by `CynlrHardwareInterface`.

---

## Phase 15: cynlr_moveit_config ✅ DONE

**Files to create:**
- `cynlr_moveit_config/config/kinematics.yaml` — KDL per arm group (swap for custom IK plugin)
- `cynlr_moveit_config/srdf/cynlr_three_arm.srdf.xacro` — 3 planning groups + `all_arms` group
- `cynlr_moveit_config/config/moveit_controllers.yaml` — maps `arm_*_jt_controller` → FollowJointTrajectory
- `cynlr_moveit_config/config/sensors_3d.yaml` — OctoMap from camera point clouds
- `cynlr_moveit_config/config/ompl_planning.yaml`

**Cross-arm coordination:** `all_arms` planning group in SRDF allows MoveIt to plan for all three arms simultaneously with inter-arm collision avoidance.

**Custom IK plugin (optional):** Implement `kinematics::KinematicsBase`, register as pluginlib plugin, reference in `kinematics.yaml`. MoveIt calls custom IK instead of KDL.

---

## Phase 16: cynlr_camera ✅ DONE

**Class:** `StereoCameraNode : rclcpp_lifecycle::LifecycleNode`

**Publishes:**
- `/camera_N/left/image_raw`, `/camera_N/right/image_raw` — `sensor_msgs/Image`
- `/camera_N/left/camera_info`, `/camera_N/right/camera_info`
- `/camera_N/points` — `sensor_msgs/PointCloud2` at ~30Hz

**MoveIt OctoMap integration:** `sensors_3d.yaml` → `PointCloudOctomapUpdater` consumes `/camera_N/points` → MoveIt plans collision-avoiding trajectories.

---

## Phase 17: Integration Tests (ros2_control) 🔲 PENDING

### SimArm smoke test
```bash
ros2 launch cynlr_bringup cynlr_system.launch.py vendor:=sim
```
→ all three state broadcasters publish, joint_state_broadcaster publishes, RViz shows robot

### JTC test
Activate `arm_left_jt_controller`, send `JointTrajectory` goal → robot moves, state broadcaster shows new positions.

### Hatch #1 test
```bash
ros2 control switch_controllers --deactivate arm_left_jt_controller --activate arm_left_direct_cmd
ros2 topic pub /arm_left_joint_cmd_direct cynlr_arm_interfaces/msg/JointCommand ...
```
→ arm holds position.

### Hatch #2 test
Activate `arm_left_nrt` → send MoveL action goal → arm moves via SimArm NRT planner.

### MoveIt test
Launch with `use_moveit:=true` → use RViz motion planning panel → trajectory flows through JTC.

### Real hardware
Replace `vendor:=sim` with `vendor:=flexiv` + supply serial numbers → same test sequence.

---

## Build Instructions

### cynlr_arm_core (Conan, WSL2)
```bash
cd /path/to/cynlr_arm_core
conan create . --profile=linux-gcc -s build_type=Release --build=missing
conan install --requires=cynlr_arm_core/0.1.0 --profile=linux-gcc -s build_type=Release \
    -g CMakeDeps --output-folder=/opt/cynlr_cmake
```

### ROS2 packages (colcon, WSL2)
```bash
source /opt/ros/jazzy/setup.bash
colcon build \
    --packages-select \
        cynlr_arm_interfaces \
        cynlr_arm_service \
        cynlr_hardware \
        cynlr_arm_description \
        cynlr_arm_controllers \
        cynlr_bringup \
    --cmake-args \
        -DCMAKE_PREFIX_PATH=/opt/cynlr_cmake \
        -DCMAKE_BUILD_TYPE=Release
```

**Note:** `cynlr_arm_description` requires `flexiv_description` to be in the workspace or on `CMAKE_PREFIX_PATH`. Clone it from https://github.com/flexivrobotics/flexiv_description (branch humble).

---

## Critical Reference Files

| Purpose | File |
|---|---|
| Hardware interface pattern | `/flexiv_ros2/flexiv_hardware/src/flexiv_hardware_interface.cpp` |
| Bit-cast pointer trick | `/flexiv_ros2/flexiv_controllers/flexiv_robot_states_broadcaster/include/flexiv_robot_states_broadcaster/flexiv_robot_states.hpp` |
| ArmInterface contract | `/Cpp_App_Test/cynlr_arm_core/include/cynlr_arm_core/arm_interface.hpp` |
| ArmState/types | `/Cpp_App_Test/cynlr_arm_core/include/cynlr_arm_core/types.hpp` |
| RPATH/SHARED build pattern | `/Cpp_App_Test/cynlr_arm_service/CMakeLists.txt` |
| Launch sequencing pattern | `/flexiv_ros2/flexiv_bringup/launch/rizon_dual.launch.py` |
| Realtime publisher pattern | `/flexiv_ros2/flexiv_controllers/flexiv_robot_states_broadcaster/src/flexiv_robot_states_broadcaster.cpp` |

---

## Testing Strategy

### Unit (Phases 1-3)
- SimArm: connect, enable, stream, fault, clear_fault, factory
- ModeManager: all valid/invalid transitions
- Framework: Google Test via Conan
- **Platform: Windows MSVC (primary); Linux before hardware deployment**
- 19/19 tests pass on Windows MSVC

### Integration — arm_service (Phase 8, PENDING)
- Full lifecycle via ROS2 service/action clients with SimArm backend
- Framework: launch_testing + Google Test
- **Platform: WSL2 (Ubuntu 24.04 + ROS2 Jazzy)**

### Integration — ros2_control (Phase 17, IN PROGRESS)

**Context:** Six new packages are unbuilt. Three of five `test_arm_service.cpp` tests are broken
because cynlr_arm_service was slimmed (MoveJ action, set_mode service, RT cmd subscriptions removed).

#### Step 1 — Fix broken integration tests

**File:** `cynlr_arm_service/tests/integration/test_arm_service.cpp`

Tests to remove (use removed functionality):
- `ConnectEnableNRTMoveJ` — calls `set_mode` srv + `move_j` action
- `RTModeJointCommandUpdatesState` — calls `set_mode` srv + publishes to `/cmd/joint`
- `MoveJCancellation` — calls `set_mode` srv + `move_j` action
- `InvalidTransitionsRejected` — partially broken (calls `set_mode`)

Tests to keep: `ClearFaultRequiresFaultMode` ✓

New tests for supervision-only role:
1. **`InvalidTransitionsRejected`** (rewrite): enable before connect → false; stop before connect → false; clear_fault before fault → false
2. **`ConnectAndDisconnect`**: `/connect` (vendor=sim) → success; `/disconnect` → success; enable after disconnect → false
3. **`ConnectEnableAndStop`**: connect+enable → success; `/stop` → success; status topic shows CONNECTED
4. **`SetToolRequiresEnabled`**: `/set_tool` before enable → false; then connect+enable, `/set_tool` → success
5. **`StatusTopicPublishes`**: connect+enable; spin 2s; verify `OperationalStatus` message received with mode=ENABLED

Remove includes: `set_mode.hpp`, `action/move_j.hpp`, `msg/joint_command.hpp`, `<rclcpp_action/rclcpp_action.hpp>`
Add includes: `srv/set_tool.hpp`, `msg/operational_status.hpp`

#### Step 2 — Build prerequisites

```bash
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers
# Clone flexiv_description into workspace
git clone https://github.com/flexivrobotics/flexiv_description.git \
  --branch humble Cpp_App_Test/flexiv_description
# Verify /opt/cynlr_cmake exists (cynlr_arm_core Conan install)
ls /opt/cynlr_cmake/lib/cmake/cynlr_arm_core/
```

#### Step 3 — Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-select cynlr_arm_interfaces cynlr_arm_service cynlr_hardware \
    cynlr_arm_description cynlr_arm_controllers cynlr_moveit_config cynlr_camera cynlr_bringup \
  --cmake-args -DCMAKE_PREFIX_PATH="/opt/cynlr_cmake;/opt/ros/jazzy" \
  --symlink-install
source install/setup.bash
```

#### Step 4 — Run updated integration tests

```bash
colcon test --packages-select cynlr_arm_service --event-handlers console_direct+
colcon test-result --all --verbose
```
Expected: 5/5 PASS.

#### Step 5 — System smoke test (vendor:=sim)

```bash
ros2 launch cynlr_bringup cynlr_system.launch.py \
  vendor:=sim sn_left:=sim0 sn_center:=sim1 sn_right:=sim2 \
  use_rviz:=false use_moveit:=false
```

Verify:
```bash
ros2 control list_controllers            # 16 controllers, 7 active, 9 inactive
ros2 topic echo /arm_left_state --once   # ArmState from CynlrStateBroadcaster
ros2 topic hz /joint_states              # ~1000Hz
ros2 topic echo /camera_0/left/image_raw --once  # StereoCameraNode synthetic frame
```

#### Step 6 — Supervision service tests

```bash
ros2 service call /arm_left/connect cynlr_arm_interfaces/srv/Connect \
  "{vendor: 'sim', num_joints: 7}"
ros2 service call /arm_left/enable cynlr_arm_interfaces/srv/Trigger {}
ros2 topic echo /arm_left/operational_status --once
ros2 service call /arm_left/set_tool cynlr_arm_interfaces/srv/SetTool \
  "{tool_id: 0, mass: 0.5, com: [0.0, 0.0, 0.1]}"
ros2 service call /arm_left/zero_ft cynlr_arm_interfaces/srv/Trigger {}
ros2 service call /arm_left/get_digital_inputs \
  cynlr_arm_interfaces/srv/GetDigitalInputs {}
```

#### Step 7 — Escape hatch switching

**Hatch #1 (direct joint command):**
```bash
ros2 control switch_controllers \
  --deactivate arm_left_jt_controller --activate arm_left_direct_cmd --strict
ros2 topic pub /arm_left/joint_cmd_direct \
  cynlr_arm_interfaces/msg/JointCommand \
  "{mode: 0, joint_position: [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0]}" --once
```

**Hatch #2 (NRT passthrough — arm's own planner):**
```bash
ros2 control switch_controllers \
  --deactivate arm_left_jt_controller --activate arm_left_nrt --strict
ros2 action send_goal /arm_left/move_j cynlr_arm_interfaces/action/MoveJ \
  "{target_positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], max_joint_vel: 0.5}"
```

**Hatch #3 (Cartesian):**
```bash
ros2 control switch_controllers \
  --deactivate arm_left_jt_controller --activate arm_left_cartesian --strict
ros2 topic pub /arm_left/cartesian_cmd \
  cynlr_arm_interfaces/msg/CartesianCommand \
  "{pose: [0.4, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0], wrench: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --once
```

### System (manual, hardware)
- Real Flexiv arm (Ethernet)
- `ros2 launch cynlr_bringup cynlr_system.launch.py vendor:=flexiv sn_left:=<SN> ...`
- FlexivArm connect → enable → move via JTC → switch to hatch → move via NRT
- RT streaming at 1kHz; `CARTESIAN_MOTION_FORCE` and `JOINT_TORQUE` Linux-only
