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

## Phase 8: Integration Tests — N/A (superseded)

`cynlr_arm_service` was deleted before this phase was implemented. Integration testing
for the NRT supervision layer is now covered by Phase 17 (CynlrArmNode).

---

## Architecture Pivot (2026-04-23): ros2_control Multi-Arm System

At this point, the architecture expanded from a standalone single-arm service to a full ros2_control-based multi-arm system. The initial implementation (phases 9–16 below) was then refined through three subsequent commits that significantly restructured the ros2_control layer. See "Architecture Refinements" below for the final state.

**Initial key design decisions:**
- Three separate hardware plugin instances (one per arm prefix) in a single controller manager
- NaN sentinel in all command buffers — `write()` skips sending if any command is NaN
- `vendor:=sim` → `SimArm`, `vendor:=flexiv` → real hardware, zero code change

---

## Phase 9: cynlr_hardware — ros2_control Plugin ✅ DONE (superseded — see refinements)

Initial hardware plugin `CynlrHardwareInterface`. Exported `ArmState*` and `ArmInterface*`
via bit_cast trick through `double*` StateInterface slots. Later replaced by `cynlr_robot`.

---

## Phase 10: cynlr_arm_description — Three-Arm URDF ✅ DONE

Per-arm macro `cynlr_rizon7_macro.urdf.xacro` + top-level `cynlr_arm_system.urdf.xacro`.
Later: URDF generation moved to launch time (OpaqueFunction pattern), driven by
`cynlr_system_config.yaml`. No static URDF file in install tree.

---

## Phase 11: cynlr_arm_controllers — Four Controller Plugins ✅ DONE (superseded — see refinements)

Initially: `CynlrStateBroadcaster`, `CynlrDirectCommandController`, `CynlrNrtPassthroughController`, `CynlrCartesianController`. After refinements, only `direct_cmd` and `cartesian` remain.

---

## Phase 12: cynlr_arm_interfaces — Extended ✅ DONE

`cynlr_arm_interfaces` package with all messages, services, actions in use. The `SwitchController.srv` that was planned was not ultimately added — controller switching uses `ros2 control switch_controllers` directly.

---

## Phase 13: cynlr_bringup — Launch Files + Config YAML ✅ DONE

`cynlr_system.launch.py` with `cynlr_system_config.yaml` as single source of truth.
Launch arg is `config_file:=<path>` (not per-arm vendor/serial args).

---

## Phase 14: cynlr_arm_service — Deleted ✅ DONE

Initially slimmed to a supervision node; then fully deleted in the single-connection
architecture refactor. All supervision ops (clear_fault, set_tool, zero_ft) and NRT actions
(move_l/j/ptp) moved to `CynlrArmNode`.

---

## Phase 15: cynlr_moveit_config ✅ DONE

KDL per arm, `all_arms` group, `moveit_controllers.yaml` maps to `arm_*_jt_controller`,
OctoMap from `/camera_0/points`.

---

## Phase 16: cynlr_camera ✅ DONE

`StereoCameraNode` publishes synthetic frames by default. Override `open_camera` /
`close_camera` / `grab_frame` to add real hardware.

---

## Architecture Refinements (2026-04-23 → 2026-04-29)

Three commits after the initial build substantially restructured the ros2_control layer.
The table below shows what changed:

| Old | New |
|---|---|
| `cynlr_hardware` (`CynlrHardwareInterface`) | `cynlr_robot` (`CynlrRobotInterface` + `CynlrArmRegistry` + `CynlrArmHandle`) |
| `cynlr_arm_service` supervision node | **Deleted** |
| `CynlrStateBroadcaster` controller plugin | **Deleted** — state publishing in `CynlrArmNode` |
| `CynlrNrtPassthroughController` plugin | **Deleted** — NRT actions in `CynlrArmNode` |
| `CynlrArmServices` plugin | **Deleted** — NRT services in `CynlrArmNode` |
| 4 controller plugins | 2 remain: `direct_cmd`, `cartesian` |
| `ArmInterface*` bit-cast via StateInterface | `CynlrArmRegistry` process-local singleton |
| Separate `ros2_control_node` process | `cynlr_main` container: 3-thread (cm_executor, cm_update_thread, arm_thread) |
| Per-arm `vendor`/`sn_*` launch args | `cynlr_system_config.yaml` single source of truth |
| `ros2_control_node` drives CM update | `cm_update_thread` in `cynlr_main` at 500 Hz |

**Reason for single-connection architecture:** The original dual-connection design (one RT
connection in `CynlrHardwareInterface`, one NRT in `cynlr_arm_service`) had a race condition
and required managing two separate lifecycles. The new design has one `ArmInterface` instance
per arm, owned by `CynlrRobotInterface`. `CynlrArmNode` accesses it through the handle/registry
— no second SDK connection.

**Reason for `cynlr_main` instead of `ros2_control_node`:** In Jazzy, `ControllerManager`
does not self-schedule its update loop. A dedicated thread calling `cm->read/update/write`
is required — the same pattern as `ros2_control_node`. Embedding it in `cynlr_main` also
allows `CynlrArmNode` to share the process and access the registry singleton.

### Current package set (all built ✅)

| Package | Role |
|---|---|
| `cynlr_arm_core` | CMake, pure C++20 — `ArmInterface`, `SimArm`, `FlexivArm` |
| `cynlr_arm_interfaces` | ROS2 IDL — messages, services, actions |
| `cynlr_robot` | Hardware plugin + `CynlrArmRegistry` + `CynlrArmHandle` |
| `cynlr_arm_node` | `CynlrArmNode` + `cynlr_main` executable |
| `cynlr_arm_controllers` | `direct_cmd`, `cartesian` plugins |
| `cynlr_arm_description` | URDF/xacro (generated at launch from config) |
| `cynlr_moveit_config` | MoveIt SRDF + config |
| `cynlr_bringup` | Launch file + config YAMLs |
| `flexiv_description` | Flexiv mesh/kinematics (humble branch submodule) |

---

## Phase 17: Integration Tests ⏳ IN PROGRESS

### Hardware verified (single-arm, 2026-04-29) ✅

```bash
ros2 launch cynlr_bringup cynlr_system.launch.py \
  config_file:=$(ros2 pkg prefix cynlr_bringup)/share/cynlr_bringup/config/cynlr_single_arm_config.yaml \
  use_rviz:=false use_moveit:=false
```

Result: `joint_state_broadcaster` + `arm_left_jt_controller` activate, Flexiv Rizon4s enters
`RT_JOINT_POSITION`, `direct_cmd` and `cartesian` load inactive. ✅

### Known open issue: WSL2 multi-arm jitter

With three arms (left=real, center+right=sim), `arm_center`/`arm_right` JT controllers
may time out during activation. Root cause: Flexiv's `stream_command` blocks the 500 Hz
`cm_update_thread` long enough for the JTC goal tolerance timer to expire. Not present on
a real-time OS.

### Remaining test steps

1. **SimArm smoke test (3-arm)**

```bash
# Edit cynlr_system_config.yaml: set all three arms to vendor: sim
ros2 launch cynlr_bringup cynlr_system.launch.py use_rviz:=false use_moveit:=false
ros2 control list_controllers   # expect 10 controllers: 4 active, 6 inactive
ros2 topic hz /joint_states     # ~500 Hz
ros2 topic echo /arm_left_arm_state --once
```

2. **Escape hatch #1 — direct joint command**

```bash
ros2 control switch_controllers \
  --deactivate arm_left_jt_controller --activate arm_left_direct_cmd --strict
ros2 topic pub /arm_left_joint_cmd_direct \
  cynlr_arm_interfaces/msg/JointCommand \
  "{mode: 0, joint_position: [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0]}" --once
```

3. **Escape hatch #2 — Cartesian**

```bash
ros2 control switch_controllers \
  --deactivate arm_left_jt_controller --activate arm_left_cartesian --strict
ros2 topic pub /arm_left_cartesian_cmd \
  cynlr_arm_interfaces/msg/CartesianCommand \
  "{cartesian_pose: [0.4, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0], wrench: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --once
```

4. **NRT actions (CynlrArmNode)**

```bash
ros2 action send_goal /arm_left_move_j cynlr_arm_interfaces/action/MoveJ \
  "{target_positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], max_joint_vel: 0.5}"
```

5. **MoveIt test**

```bash
ros2 launch cynlr_bringup cynlr_system.launch.py use_moveit:=true use_rviz:=true
# Use RViz MotionPlanning panel → plan → execute
```

6. **Real hardware — three arms**

Requires resolving the WSL2 multi-arm jitter issue (non-RT jitter from `stream_command`).
On a real-time Linux host this should work without changes.

---

## Build Instructions

See `BUILD.md` for the full step-by-step guide. Summary:

### flexiv_rdk (submodule, one-time)
```bash
git submodule update --init --recursive
cmake -S flexiv_rdk -B flexiv_rdk/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/cynlr_software/rdk_install
cmake --build flexiv_rdk/build -j$(nproc) && cmake --install flexiv_rdk/build
```

### cynlr_arm_core (CMake, one-time)
```bash
cmake -S cynlr_arm_core -B cynlr_arm_core/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$HOME/cynlr_software/rdk_install" \
  -DCMAKE_INSTALL_PREFIX=$HOME/cynlr_software/cynlr_install
cmake --build cynlr_arm_core/build -j$(nproc) && cmake --install cynlr_arm_core/build
```

### ROS2 packages (colcon, WSL2)
```bash
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-select \
    flexiv_description cynlr_arm_interfaces cynlr_robot cynlr_arm_node \
    cynlr_arm_controllers cynlr_arm_description cynlr_moveit_config cynlr_bringup \
  --cmake-args \
    -DCMAKE_PREFIX_PATH="$HOME/cynlr_software/cynlr_install;/opt/ros/jazzy"
source install/setup.bash
```

---

## Critical Reference Files

| Purpose | File |
|---|---|
| Hardware plugin | `Cpp_App_Test/cynlr_robot/src/cynlr_robot_interface.cpp` |
| Arm handle / registry | `Cpp_App_Test/cynlr_robot/include/cynlr_robot/cynlr_arm_handle.hpp` |
| Main executable (3-thread pattern) | `Cpp_App_Test/cynlr_arm_node/src/cynlr_main.cpp` |
| Arm node (publishers, services, actions) | `Cpp_App_Test/cynlr_arm_node/src/cynlr_arm_node.cpp` |
| ArmInterface contract | `Cpp_App_Test/cynlr_arm_core/include/cynlr_arm_core/arm_interface.hpp` |
| ArmState/types | `Cpp_App_Test/cynlr_arm_core/include/cynlr_arm_core/types.hpp` |
| System config (3-arm) | `Cpp_App_Test/cynlr_bringup/config/cynlr_system_config.yaml` |
| System config (1-arm) | `Cpp_App_Test/cynlr_bringup/config/cynlr_single_arm_config.yaml` |
| Hardware interface pattern (reference) | `flexiv_ros2/flexiv_hardware/src/flexiv_hardware_interface.cpp` |

---

## Testing Strategy

### Unit (Phases 1-3)
- SimArm: connect, enable, stream, fault, clear_fault, factory
- ModeManager: all valid/invalid transitions
- Framework: Google Test via Conan
- **Platform: Windows MSVC (primary); Linux before hardware deployment**
- 19/19 tests pass on Windows MSVC

### Integration — arm_service (Phase 8)

No longer applicable. `cynlr_arm_service` was deleted. NRT services and actions are hosted
by `CynlrArmNode` inside `cynlr_arm_node`; no separate integration test package exists yet.

### Integration — ros2_control (Phase 17, IN PROGRESS)

See Phase 17 test steps above.

**Single-arm hardware verified ✅** (2026-04-29, Flexiv Rizon4s):
- JSB + JT controller activate, arm enters `RT_JOINT_POSITION`
- `direct_cmd` and `cartesian` load inactive

**Remaining:** SimArm 3-arm smoke, escape hatch switching, NRT actions, MoveIt, multi-arm hardware (blocked by WSL2 jitter)

### System (manual, hardware)
- `ros2 launch cynlr_bringup cynlr_system.launch.py config_file:=<path> use_rviz:=true`
- Edit `cynlr_system_config.yaml` to set `vendor: flexiv` and real serial numbers
- `CARTESIAN_MOTION_FORCE` and `JOINT_TORQUE` streaming: Linux only
