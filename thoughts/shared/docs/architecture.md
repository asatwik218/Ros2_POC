# CynLR Software Architecture

CynLR is a physical intelligence device: three Flexiv Rizon 7-DOF arms and stereo cameras
on a shared frame. The software is designed so you can freely mix motion approaches —
MoveIt, the arm's own planner, a learning policy, force control — without ever rewriting
the hardware layer. Everything from the arm SDK upward is swappable at runtime.

---

## Table of Contents

1. [Layer diagram](#layer-diagram)
2. [Package overview](#package-overview)
3. [cynlr_arm_core](#cynlr_arm_core)
4. [cynlr_arm_interfaces](#cynlr_arm_interfaces)
5. [cynlr_hardware](#cynlr_hardware)
6. [cynlr_arm_description](#cynlr_arm_description)
7. [cynlr_arm_controllers](#cynlr_arm_controllers)
8. [cynlr_arm_service](#cynlr_arm_service)
9. [cynlr_moveit_config](#cynlr_moveit_config)
10. [cynlr_camera](#cynlr_camera)
11. [cynlr_bringup](#cynlr_bringup)
12. [How the packages connect](#how-the-packages-connect)
13. [The four motion modes](#the-four-motion-modes)
14. [Switching modes at runtime](#switching-modes-at-runtime)
15. [Configuration reference](#configuration-reference)
16. [Build and run](#build-and-run)

---

## Layer diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  Your code / AI policy / BT engine                                          │
│  publishes ROS topics, calls ROS services, sends action goals               │
└─────────────────────────────────────┬───────────────────────────────────────┘
                                      │
          ┌───────────────────────────┼───────────────────────────┐
          │                           │                           │
   ┌──────▼──────┐            ┌───────▼───────┐          ┌───────▼──────────┐
   │   MoveIt    │            │  cynlr_arm_   │          │  cynlr_arm_      │
   │  move_group │            │  service      │          │  camera          │
   │  (optional) │            │  (supervision)│          │  (stereo pair)   │
   └──────┬──────┘            └───────────────┘          └──────────────────┘
          │ FollowJointTrajectory
          │
┌─────────▼─────────────────────────────────────────────────────────────────┐
│  ros2_control  controller_manager  (1000 Hz RT loop)                       │
│                                                                             │
│  ┌──────────────────┐  ┌───────────────────┐  ┌─────────────────────────┐ │
│  │ JointTrajCtrl    │  │ DirectCmdCtrl      │  │ CartesianCtrl           │ │
│  │ (default/MoveIt) │  │ (escape hatch #1)  │  │ (escape hatch #3)       │ │
│  └──────────────────┘  └───────────────────┘  └─────────────────────────┘ │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │  NrtPassthroughCtrl (escape hatch #2)  — MoveL/J/PTP action servers  │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │  CynlrStateBroadcaster  — publishes ArmState, TCP, FT wrenches        │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────┬─────────────────────────────────────────┘
                                  │ StateInterface / CommandInterface
┌─────────────────────────────────▼─────────────────────────────────────────┐
│  cynlr_hardware  CynlrHardwareInterface  (one instance per arm)            │
│  on_activate: connect → enable │ read: get_state → fill buffers           │
│  write: dispatch by mode → stream_command / gpio                           │
└─────────────────────────────────┬─────────────────────────────────────────┘
                                  │  C++ API (no ROS)
┌─────────────────────────────────▼─────────────────────────────────────────┐
│  cynlr_arm_core   ArmInterface   (vendor-agnostic, pure C++20)             │
│  FlexivArm → Flexiv RDK        SimArm → integrator / FK                   │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## Package overview

| Package | Type | Purpose |
|---|---|---|
| `cynlr_arm_core` | Conan (C++, no ROS) | Vendor-agnostic arm API + implementations |
| `cynlr_arm_interfaces` | ROS2 ament | ROS message/service/action definitions |
| `cynlr_hardware` | ROS2 ament, pluginlib | ros2_control hardware plugin (SystemInterface) |
| `cynlr_arm_description` | ROS2 ament | URDF/xacro for three-arm system |
| `cynlr_arm_controllers` | ROS2 ament, pluginlib | Four controller plugins |
| `cynlr_arm_service` | ROS2 ament | Supervision node (fault, lifecycle, tool config) |
| `cynlr_moveit_config` | ROS2 ament | MoveIt SRDF, kinematics, planner config |
| `cynlr_camera` | ROS2 ament | Stereo camera lifecycle node |
| `cynlr_bringup` | ROS2 ament | Launch files + controller YAML |

---

## cynlr_arm_core

**Language:** Pure C++20, no ROS, built with Conan.  
**Location:** `cynlr_arm_core/`

This is the hardware abstraction layer. Everything above it (ros2_control, MoveIt, your AI
policy) talks to an `ArmInterface*`. There is no Flexiv-specific code anywhere above this
package.

### ArmInterface

The central abstract base. Any arm implementation must implement all of these:

```
Lifecycle    connect / disconnect / enable / stop
State        get_state → ArmState  (joint pos/vel/torque, TCP pose, FT wrench)
             is_connected / has_fault / clear_fault / run_auto_recovery
NRT motion   move_l  (Cartesian linear)
             move_j  (joint-space, via arm's built-in planner)
             move_ptp (point-to-point Cartesian, via arm's built-in planner)
             is_motion_complete → bool (poll until done)
RT streaming start_streaming(mode) / stream_command(cmd) / stop_streaming
             modes: JOINT_POSITION | JOINT_TORQUE | CARTESIAN_MOTION_FORCE
Tool config  set_tool / zero_ft_sensor
Capability   supported_features → []string   (plus dynamic_cast for compile-time checks)
```

### Capability interfaces

Optional mixins. Dynamic-cast the `ArmInterface*` to check support:

| Interface | Methods | Used by |
|---|---|---|
| `DigitalIOControllable` | `set_digital_outputs` / `get_digital_inputs` | `cynlr_arm_service` GPIO services, `CynlrHardwareInterface` |
| `ForceControllable` | `set_force_control_axis` / `set_force_control_frame` / `set_passive_force_control` | User code via service calls |
| `ImpedanceConfigurable` | `set_cartesian_impedance` / `set_joint_impedance` | User code |
| `NullSpaceConfigurable` | `set_null_space_posture` / `set_null_space_objectives` | User code |

### ArmState

The complete snapshot returned by `get_state()` at 1 kHz:

```cpp
struct ArmState {
    array<double,7> joint_positions;       // rad
    array<double,7> joint_velocities;      // rad/s
    array<double,7> joint_torques;         // Nm
    array<double,7> joint_torques_external;// Nm
    array<double,7> tcp_pose;              // [x,y,z, qw,qx,qy,qz]
    array<double,6> tcp_velocity;          // [vx,vy,vz, wx,wy,wz]
    array<double,6> ft_sensor_raw;         // [fx,fy,fz, tx,ty,tz] N/Nm
    array<double,6> ext_wrench_in_tcp;
    array<double,6> ext_wrench_in_world;
    bool fault, operational, estopped;
};
```

### Implementations

**`FlexivArm`** — wraps `flexiv::rdk::Robot`. Implements all four capability interfaces.
RT streaming only on Linux (`#ifdef __linux__`); position streaming works on both platforms.

**`SimArm`** — pure software simulation. Joint integrator (`pos += vel * dt`), simple FK for
TCP. Supports `ForceControllable` and `NullSpaceConfigurable`. Has `inject_fault()` and
`set_noise_stddev()` for testing.

### create_arm factory

```cpp
// vendor="sim"    → SimArm
// vendor="flexiv" → FlexivArm
// unknown         → nullptr
std::unique_ptr<ArmInterface> create_arm(const ArmConfig& config);
```

---

## cynlr_arm_interfaces

**Language:** ROS2 IDL (`.msg`, `.srv`, `.action`).  
**Location:** `cynlr_arm_interfaces/`

Defines every ROS interface used by the system. No C++ logic.

### Messages

| Message | Fields | Published by |
|---|---|---|
| `ArmState` | All fields from `cynlr::arm::ArmState` + header | `CynlrStateBroadcaster` |
| `JointCommand` | mode, joint_position[7], joint_velocity[7], joint_torque[7] | Your code → `CynlrDirectCommandController` |
| `CartesianCommand` | cartesian_pose[7], wrench[6] | Your code → `CynlrCartesianController` |
| `FaultEvent` | fault, clearable, message, error_code | `cynlr_arm_service` |
| `OperationalStatus` | connected, operational, estopped, mode | `cynlr_arm_service` |

### Services

| Service | Fields | Handled by |
|---|---|---|
| `Connect` | vendor, serial_number, ip_address, num_joints → success, message | `cynlr_arm_service` |
| `Trigger` | (empty) → success, message | enable / stop / clear_fault / zero_ft / auto_recovery |
| `SetTool` | mass_kg, com[3], inertia[6], tcp_pose[7] → success | `cynlr_arm_service` |
| `SetDigitalOutputs` | indices[], values[] → success | `cynlr_arm_service` |
| `GetDigitalInputs` | (empty) → success, values[] | `cynlr_arm_service` |
| `SwitchController` | arm ("left"/"center"/"right"), activate[], deactivate[], strictness → success | Your code → ros2_control |

### Actions

| Action | Goal | Feedback | Result |
|---|---|---|---|
| `MoveL` | target_pose[7], max_linear_vel, max_linear_acc | current_pose[7], distance_remaining | success, message, final_pose[7] |
| `MoveJ` | target_positions[7], max_joint_vel, max_joint_acc | current_positions[7], position_error[7] | success, message, final_positions[7] |
| `MovePTP` | target_pose[7], max_joint_vel, max_joint_acc | current_pose[7], distance_remaining | success, message, final_pose[7] |

---

## cynlr_hardware

**Language:** C++20, pluginlib plugin.  
**Location:** `cynlr_hardware/`  
**Plugin class:** `cynlr_hardware/CynlrHardwareInterface`

This is the bridge between ros2_control's 1 kHz loop and `cynlr_arm_core`. There is one
instance per arm (three total). They all run in the same controller_manager thread.

### What it does

**Startup (`on_init`):**
- Reads `prefix`, `vendor`, `serial_number`, `ip_address` from the URDF `<hardware>` block
- Calls `create_arm(config)` — creates the arm object but does NOT connect
- Fills all command buffers with `NaN` (sentinel: "no command yet, don't send")

**On activate (`on_activate`):**
- Calls `arm_->connect() → clear_fault() → enable()`
- Seeds `cmd_pos_` from current position so the arm does not lurch on first write

**On deactivate (`on_deactivate`):**
- Calls `arm_->stop_streaming() → stop() → disconnect()`

**Read (called every 1 ms):**
- `arm_->get_state()` → copies into `hw_pos_`, `hw_vel_`, `hw_eff_`
- Copies full `ArmState` struct into `arm_state_`
- Reads GPIO inputs

**Write (called every 1 ms):**
- Dispatches by `active_mode_`:
  - `POSITION` → `stream_command({JOINT_POSITION, cmd_pos_, cmd_vel_})`
  - `EFFORT` → `stream_command({JOINT_TORQUE, cmd_eff_})`
  - `CARTESIAN` → `stream_command({CARTESIAN_MOTION_FORCE, cmd_cart_[0..6], cmd_cart_[7..12]})`
- Sets GPIO outputs only when values have changed (diff tracking)
- Skips any send if any commanded value is `NaN`

### The pointer trick

Controllers need access to data that doesn't fit in a `double`. `CynlrHardwareInterface`
solves this by storing a C++ pointer inside a StateInterface slot:

```cpp
// In export_state_interfaces():
// Store ArmState* as raw bits inside a double slot
reinterpret_cast<double*>(&arm_state_ptr_)

// In CynlrStateBroadcaster::on_activate():
// Recover it safely
arm_state_ptr_ = bit_cast<cynlr::arm::ArmState*>(
    state_interfaces_[idx].get_value());
```

The same trick is used for `ArmInterface*` so `CynlrNrtPassthroughController` can call
`move_l/j/ptp()` without any extra IPC.

### StateInterfaces exported (per arm)

| Name | Type | Content |
|---|---|---|
| `{prefix}joint{1-7}/position` | double | joint angle (rad) |
| `{prefix}joint{1-7}/velocity` | double | joint velocity (rad/s) |
| `{prefix}joint{1-7}/effort` | double | joint torque (Nm) |
| `{prefix}ft_sensor/raw_{0-5}` | double | raw FT sensor [fx..tz] |
| `{prefix}ft_sensor/ext_tcp_{0-5}` | double | external wrench in TCP |
| `{prefix}ft_sensor/ext_world_{0-5}` | double | external wrench in world |
| `{prefix}tcp/pose_{0-6}` | double | TCP pose [x,y,z,qw,qx,qy,qz] |
| `{prefix}tcp/vel_{0-5}` | double | TCP velocity |
| `{prefix}cynlr_arm_state/full_state_ptr` | double* | ArmState* (bit_cast) |
| `{prefix}cynlr_arm_ctrl/arm_interface_ptr` | double* | ArmInterface* (bit_cast) |
| `{prefix}gpio/in_{0-17}` | double | digital inputs |

### CommandInterfaces exported (per arm)

| Name | Type | Written by |
|---|---|---|
| `{prefix}joint{1-7}/position` | double | JTC / DirectCmdCtrl |
| `{prefix}joint{1-7}/velocity` | double | DirectCmdCtrl |
| `{prefix}joint{1-7}/effort` | double | (future torque controller) |
| `{prefix}cartesian_cmd/pose_{0-6}` | double | CynlrCartesianController |
| `{prefix}cartesian_cmd/wrench_{0-5}` | double | CynlrCartesianController |
| `{prefix}gpio/out_{0-17}` | double | (future GPIO controller) |

---

## cynlr_arm_description

**Language:** xacro (XML macro).  
**Location:** `cynlr_arm_description/`

Defines what the robot looks like: link geometry, mass, inertia, joint limits, and — crucially —
the `<ros2_control>` block that tells the controller manager which hardware plugin to load for
each arm.

### Files

**`urdf/cynlr_rizon7_macro.urdf.xacro`** — macro for a single arm.

Parameters: `prefix`, `serial_number`, `vendor`, `parent`, `*origin`

It includes `$(find flexiv_description)/urdf/rizon7_macro.urdf.xacro` for Flexiv's link
geometry and kinematics, then wraps it with the CynLR `<ros2_control>` block. This means
geometry changes (mesh files, inertia values) come from Flexiv's package; only the ros2_control
wiring is in CynLR's code.

**`urdf/cynlr_arm_system.urdf.xacro`** — top-level. Instantiates all three arms:

```
Left arm:   xyz="-0.5  0.0  0.9"  rpy="0 0 0"         (prefix: arm_left_)
Center arm: xyz=" 0.0  0.0  1.2"  rpy="0 0 0"         (prefix: arm_center_)
Right arm:  xyz=" 0.5  0.0  0.9"  rpy="0 0 π"         (prefix: arm_right_, facing inward)
```

> **Note:** Update the `xyz`/`rpy` values to match your physical mounting geometry before
> running on real hardware.

### How prefixes work

Prefix `arm_left_` means:
- Links are named `arm_left_base_link`, `arm_left_link1` … `arm_left_flange`
- Joints are named `arm_left_joint1` … `arm_left_joint7`
- All ros2_control interfaces are named `arm_left_joint1/position` etc.
- All controllers for this arm have `arm_left_` in their name
- ROS topics from the state broadcaster are under `/arm_left_*`

All three arms are identical except for the prefix, base pose, and serial number.

### Dependency

Requires `flexiv_description` (https://github.com/flexivrobotics/flexiv_description, branch
`humble`) to be present in the colcon workspace. It provides the Rizon 7 URDF mesh files and
kinematic chain.

---

## cynlr_arm_controllers

**Language:** C++20, pluginlib.  
**Location:** `cynlr_arm_controllers/`

Four controller plugins, all registered in `cynlr_controllers_plugin.xml`. One package,
four shared libraries. All use `realtime_tools::RealtimeBuffer` or
`realtime_tools::RealtimePublisher` to cross the RT boundary safely.

---

### CynlrStateBroadcaster (always active)

**Plugin:** `cynlr_state_broadcaster/CynlrStateBroadcaster`

Claims the `full_state_ptr` StateInterface (read-only). In `on_activate()` it recovers
the `ArmState*` from the double slot via `bit_cast`, then holds that pointer for the
lifetime of the controller.

On every `update()` call (1 kHz):
- Publishes `/{prefix}tcp_pose` (PoseStamped) and all wrench topics (WrenchStamped) — high rate
- Every N ticks (controlled by `publish_rate` param, default 100 Hz): publishes
  `/{prefix}arm_state` (ArmState) — full struct with all joints and sensor data

**Topics published:**

| Topic | Type | Rate |
|---|---|---|
| `/{prefix}arm_state` | `cynlr_arm_interfaces/ArmState` | `publish_rate` Hz (configurable) |
| `/{prefix}tcp_pose` | `geometry_msgs/PoseStamped` | 1000 Hz |
| `/{prefix}ft_raw` | `geometry_msgs/WrenchStamped` | 1000 Hz |
| `/{prefix}ext_wrench_tcp` | `geometry_msgs/WrenchStamped` | 1000 Hz |
| `/{prefix}ext_wrench_world` | `geometry_msgs/WrenchStamped` | 1000 Hz |

---

### JointTrajectoryController (default motion path)

**Plugin:** `joint_trajectory_controller/JointTrajectoryController` (standard ROS2 package)

Claims joint position + velocity command interfaces. Receives
`trajectory_msgs/JointTrajectory` goals via the FollowJointTrajectory action server at
`/{prefix}jt_controller/follow_joint_trajectory`. This is the controller MoveIt talks to.

---

### CynlrDirectCommandController — Escape Hatch #1

**Plugin:** `cynlr_direct_command_controller/CynlrDirectCommandController`

The simplest possible escape hatch. Claims joint position + velocity command interfaces.
Subscribes to `/{prefix}joint_cmd_direct` (`JointCommand`) with BEST_EFFORT, volatile QoS
(drop old messages). In `on_activate()` it seeds positions from current state so the arm
holds still until a command arrives.

In `update()`: reads from a `RealtimeBuffer` (written by the NRT subscriber callback) and
copies `joint_position[7]` and `joint_velocity[7]` directly to the command interfaces.

**Use this when:** you have a custom planner, learning policy, or teleoperation node that
computes joint commands. Your node publishes at up to 1 kHz; the controller passes them
straight to the hardware with zero additional processing.

**Topic subscribed:**

```
/{prefix}joint_cmd_direct    cynlr_arm_interfaces/msg/JointCommand
```

---

### CynlrNrtPassthroughController — Escape Hatch #2

**Plugin:** `cynlr_nrt_passthrough_controller/CynlrNrtPassthroughController`

Hosts three action servers. Claims **no command interfaces** — it uses the arm's own
built-in trajectory planner instead of commanding joints directly.

In `on_activate()` it recovers the `ArmInterface*` from the `arm_interface_ptr` StateInterface
via `bit_cast`. When a goal arrives, it spawns a detached thread that:
1. Calls `arm_->move_l/j/ptp()` (NRT, returns after sending the command)
2. Polls `arm_->is_motion_complete()` every 20 ms
3. Publishes `get_state()` as feedback
4. Calls `gh->succeed()` or `gh->abort()` when done

**Use this when:** you want high-level "go to pose" commands without worrying about
trajectory parameterization, speed limits, or interpolation. The arm's internal planner
generates a smooth, dynamically feasible path.

**Action servers:**

```
/{prefix}move_l    cynlr_arm_interfaces/action/MoveL    (Cartesian linear)
/{prefix}move_j    cynlr_arm_interfaces/action/MoveJ    (joint-space)
/{prefix}move_ptp  cynlr_arm_interfaces/action/MovePTP  (Cartesian PTP)
```

---

### CynlrCartesianController — Escape Hatch #3

**Plugin:** `cynlr_cartesian_controller/CynlrCartesianController`

Claims the 13 GPIO-type command interfaces (`cartesian_cmd/pose_0..6`,
`cartesian_cmd/wrench_0..5`). Subscribes to `/{prefix}cartesian_cmd` (`CartesianCommand`)
with BEST_EFFORT QoS. In `update()` writes pose and wrench to the command interfaces.

The hardware `write()` detects `CARTESIAN` mode and calls:
```cpp
arm_->stream_command({CARTESIAN_MOTION_FORCE, pose[7], wrench[6]})
```

On `on_deactivate()` fills all 13 interfaces with `NaN` so the hardware plugin stops sending.

**Use this when:** you are running a force-controlled task (insertion, polishing, assembly),
visual servoing, or a Cartesian-space learning policy. Your node computes desired pose and
contact wrench, publishes at up to 1 kHz.

**Topic subscribed:**

```
/{prefix}cartesian_cmd    cynlr_arm_interfaces/msg/CartesianCommand
```

---

## cynlr_arm_service

**Language:** C++20, ROS2 lifecycle node.  
**Location:** `cynlr_arm_service/`

**Role:** Supervision only. Fault monitoring and NRT arm configuration.

This node holds its own `ArmInterface` connection — separate from the one inside
`CynlrHardwareInterface`. The Flexiv SDK supports concurrent NRT connections alongside the
1 kHz RT connection. This node uses its connection exclusively for configuration calls that
don't fit into the ros2_control model.

**What it is NOT:** it does not drive the RT loop (that's the controller manager), publish
arm state (that's `CynlrStateBroadcaster`), or host motion actions (that's
`CynlrNrtPassthroughController`).

### ModeManager

Tracks connection state with a validated state machine:

```
IDLE → CONNECTED → ENABLED → IDLE / FAULT
                   FAULT   → CONNECTED (after clear_fault)
```

### Services exposed

All under the node namespace (e.g. `/arm_service_left/`):

| Service | Type | What it does |
|---|---|---|
| `connect` | `Connect` | Connects the NRT arm instance |
| `disconnect` | `Trigger` | Disconnects |
| `enable` | `Trigger` | Enables servos |
| `stop` | `Trigger` | Stops current motion |
| `clear_fault` | `Trigger` | Clears fault, transitions FAULT→CONNECTED |
| `set_tool` | `SetTool` | Updates tool mass/CoM/inertia/TCP offset |
| `zero_ft_sensor` | `Trigger` | Zeros the FT sensor in current pose |
| `auto_recovery` | `Trigger` | Runs built-in auto recovery sequence |
| `set_digital_outputs` | `SetDigitalOutputs` | Sets digital output ports |
| `get_digital_inputs` | `GetDigitalInputs` | Reads digital input ports |

### Topics published

| Topic | Type | Rate |
|---|---|---|
| `fault` | `FaultEvent` | On fault detection |
| `status` | `OperationalStatus` | 5 Hz (fault poll timer) |

### Fault monitoring

A 200 ms wall timer calls `arm_->has_fault()`. On first detection it transitions
`ModeManager` to FAULT and publishes a `FaultEvent`. The ros2_control hardware plugin
monitors the same flag independently — the service node's arm connection is for NRT
configuration only, not for the RT loop.

---

## cynlr_moveit_config

**Language:** xacro, YAML.  
**Location:** `cynlr_moveit_config/`

Configuration files for MoveIt's `move_group` node. No compiled code.

### Planning groups (SRDF)

| Group | Chain | Use |
|---|---|---|
| `arm_left_arm` | `arm_left_base_link` → `arm_left_flange` | Single left arm planning |
| `arm_center_arm` | `arm_center_base_link` → `arm_center_flange` | Single center arm planning |
| `arm_right_arm` | `arm_right_base_link` → `arm_right_flange` | Single right arm planning |
| `all_arms` | group-of-groups | Coordinated three-arm planning with inter-arm collision avoidance |

### Named states (per arm)

- `{prefix}home` — joints at `[0, -40°, 0, 90°, 0, 40°, 0]` (upright, clear of work surface)
- `{prefix}zero` — all joints at 0 (straight up)

### Config files

| File | Purpose |
|---|---|
| `srdf/cynlr_three_arm.srdf.xacro` | Robot semantic description (groups, named states, collisions) |
| `srdf/cynlr_rizon7_macro.srdf.xacro` | Per-arm SRDF macro |
| `config/kinematics.yaml` | KDL solver per arm (swap for custom IK) |
| `config/moveit_controllers.yaml` | Maps planning groups to JTC action servers |
| `config/ompl_planning.yaml` | All OMPL planners enabled, same set for all groups |
| `config/sensors_3d.yaml` | OctoMap updater from `/camera_0/points` for collision avoidance |

### Swapping the IK solver

Replace KDL with a custom plugin in `config/kinematics.yaml`:

```yaml
arm_left_arm:
  kinematics_solver: your_package/YourKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
```

No other changes needed. The hardware layer and controllers are unaffected.

---

## cynlr_camera

**Language:** C++20, ROS2 lifecycle node.  
**Location:** `cynlr_camera/`

One `StereoCameraNode` instance per physical stereo pair. Publishes images, camera info, and
a point cloud for MoveIt's OctoMap collision avoidance.

### Published topics (per camera index N)

| Topic | Type | Rate |
|---|---|---|
| `/camera_N/left/image_raw` | `sensor_msgs/Image` | `publish_rate` Hz |
| `/camera_N/right/image_raw` | `sensor_msgs/Image` | `publish_rate` Hz |
| `/camera_N/left/camera_info` | `sensor_msgs/CameraInfo` | `publish_rate` Hz |
| `/camera_N/right/camera_info` | `sensor_msgs/CameraInfo` | `publish_rate` Hz |
| `/camera_N/points` | `sensor_msgs/PointCloud2` | `publish_rate` Hz |

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `camera_index` | `0` | Numeric index, used in topic names |
| `publish_rate` | `30.0` | Hz |
| `frame_id_left` | `camera_0_left_optical` | TF frame for left images |
| `frame_id_right` | `camera_0_right_optical` | TF frame for right images |
| `calibration_url` | `""` | `camera_info_manager` URL (e.g. `file:///path/to/cal.yaml`) |

### Adding real hardware

Subclass `StereoCameraNode` and override the three virtual methods:

```cpp
class MyRealCamera : public cynlr::camera::StereoCameraNode {
protected:
    bool open_camera() override   { /* open SDK, init sensor */ return true; }
    void close_camera() override  { /* close SDK */ }
    bool grab_frame(Image& left, Image& right, PointCloud2& cloud) override {
        /* fill from SDK, return false if not ready */
        return true;
    }
};
```

The default implementation produces synthetic 1×1 frames so all topics exist immediately
without hardware.

---

## cynlr_bringup

**Language:** Python (launch), YAML.  
**Location:** `cynlr_bringup/`

Entry point for the entire system. One launch file, one controller YAML.

### cynlr_system.launch.py

**Arguments:**

| Argument | Default | Description |
|---|---|---|
| `sn_left` | `""` | Serial number of left arm |
| `sn_center` | `""` | Serial number of center arm |
| `sn_right` | `""` | Serial number of right arm |
| `vendor` | `sim` | `sim` or `flexiv` |
| `use_moveit` | `false` | Launch `move_group` node |
| `use_rviz` | `true` | Launch RViz |

**Startup sequence** (each step waits for the previous to complete):

```
1. ros2_control_node  +  robot_state_publisher     (start immediately)
2. joint_state_broadcaster                          (spawner)
3. arm_left_state_broadcaster                       (after jsb exits)
4. arm_center_state_broadcaster                     (after left_state)
5. arm_right_state_broadcaster                      (after center_state)
6. arm_left_jt_controller                           (after right_state)
7. arm_center_jt_controller                         (after left_jt)
8. arm_right_jt_controller                          (after center_jt)
9. Escape hatches (--inactive):
   arm_{left,center,right}_{direct_cmd,nrt,cartesian}  (after right_jt)
10. rviz2 + move_group                              (after all hatches loaded, conditional)
```

### cynlr_controllers.yaml

Controller manager configuration at 1000 Hz. Three instances of each controller type (left,
center, right), plus per-controller parameter blocks:
- JTC: joint names, `stopped_velocity_tolerance: 0.01`, `goal_time: 5.0`
- State broadcaster: `prefix`, `publish_rate: 100.0`
- Direct/NRT/Cartesian: `prefix`, joint names

---

## How the packages connect

The dependency graph flows in one direction: code below never imports code above.

```
cynlr_arm_core          ← no ROS, no dependencies upward
    ↑
cynlr_hardware          ← links cynlr_arm_core, implements hardware_interface::SystemInterface
    ↑
cynlr_arm_description   ← URDF references cynlr_hardware plugin by name (string in XML)
    ↑
cynlr_arm_controllers   ← links cynlr_arm_core (for types); reads StateInterfaces from hardware
    ↑
cynlr_moveit_config     ← YAML references controller action server names; no C++ deps
cynlr_arm_service       ← links cynlr_arm_core; uses cynlr_arm_interfaces
cynlr_camera            ← no cynlr_arm_core dependency; standalone
cynlr_bringup           ← wires everything together via launch + YAML
```

### Data flow at runtime (1 kHz RT loop)

```
arm hardware
    │
    ▼  arm_->get_state()
CynlrHardwareInterface::read()
    │  fills hw_pos_[], hw_vel_[], hw_eff_[], arm_state_
    │
    ▼  StateInterface slots
controller_manager distributes to controllers
    │
    ├─► CynlrStateBroadcaster::update()
    │      reads arm_state_ptr_→ publishes /arm_left_arm_state etc.
    │
    ├─► JointTrajectoryController::update()     (if active)
    │      reads hw_pos_[], hw_vel_[] → computes PID → writes cmd_pos_[]
    │
    ├─► CynlrDirectCommandController::update()  (if active)
    │      reads RealtimeBuffer (written by ROS subscriber) → writes cmd_pos_[], cmd_vel_[]
    │
    └─► CynlrCartesianController::update()      (if active)
           reads RealtimeBuffer → writes cmd_cart_[]
    │
    ▼  CommandInterface slots
CynlrHardwareInterface::write()
    │
    ▼  arm_->stream_command(cmd)
arm hardware
```

### How NrtPassthroughController fits in

`CynlrNrtPassthroughController` does not participate in the 1 kHz data flow above. It only
reads `arm_interface_ptr_` once (on activate), then drives the arm via NRT calls from a
detached thread:

```
Action client
    │  goal: MoveL {target_pose, vel_limit}
    ▼
CynlrNrtPassthroughController (detached thread)
    │  arm_ptr_->move_l(target, params)  ← NRT, returns immediately
    │  loop: arm_ptr_->is_motion_complete() every 20ms
    │        arm_ptr_->get_state() → publish feedback
    ▼  goal result: succeed / abort / cancel
```

---

## The four motion modes

| Mode | Active controller | RT/NRT | Who plans | Use case |
|---|---|---|---|---|
| **MoveIt** | `arm_*_jt_controller` | RT (JTC executes) | MoveIt OMPL | Complex multi-arm planning, collision avoidance, IK |
| **NRT planner** | `arm_*_nrt` | NRT | Arm's internal planner | Simple point-to-point moves, task sequencing |
| **Direct joint** | `arm_*_direct_cmd` | RT (1 kHz) | Your code | Learning policies, custom joint-space planners, teleoperation |
| **Cartesian** | `arm_*_cartesian` | RT (1 kHz) | Your code | Force control, visual servoing, Cartesian learning policies |

Only one controller per arm may own the command interfaces at a time. Controllers that are
loaded but inactive (`--inactive`) hold no interfaces and do not affect the other controllers.

---

## Switching modes at runtime

All switches are hot: no restart required, no hardware interruption.

### MoveIt → Direct joint commands

```bash
ros2 control switch_controllers \
  --activate arm_left_direct_cmd \
  --deactivate arm_left_jt_controller arm_left_nrt arm_left_cartesian \
  --controller-manager /controller_manager
```

Then publish:
```bash
ros2 topic pub /arm_left_joint_cmd_direct cynlr_arm_interfaces/msg/JointCommand \
  '{mode: 0, joint_position: [0.0, -0.5, 0.0, 1.2, 0.0, 0.5, 0.0]}'
```

### MoveIt → NRT arm planner (MoveJ)

```bash
ros2 control switch_controllers \
  --activate arm_left_nrt \
  --deactivate arm_left_jt_controller arm_left_direct_cmd arm_left_cartesian
```

Then send action:
```bash
ros2 action send_goal /arm_left_move_j cynlr_arm_interfaces/action/MoveJ \
  '{target_positions: [0.0, -0.5, 0.0, 1.2, 0.0, 0.5, 0.0], max_joint_vel: 0.5}'
```

### MoveIt → Cartesian force/motion control

```bash
ros2 control switch_controllers \
  --activate arm_left_cartesian \
  --deactivate arm_left_jt_controller arm_left_direct_cmd arm_left_nrt
```

Then publish:
```bash
ros2 topic pub /arm_left_cartesian_cmd cynlr_arm_interfaces/msg/CartesianCommand \
  '{cartesian_pose: [0.4, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0], wrench: [0.0, 0.0, -5.0, 0.0, 0.0, 0.0]}'
```

### Switch back to MoveIt

```bash
ros2 control switch_controllers \
  --activate arm_left_jt_controller \
  --deactivate arm_left_direct_cmd arm_left_nrt arm_left_cartesian
```

---

## Configuration reference

### Launch configurations

#### Simulation only (no hardware)

```bash
ros2 launch cynlr_bringup cynlr_system.launch.py \
  vendor:=sim \
  use_rviz:=true
```

All three arms start as SimArm instances. No network, no serial numbers needed.

#### Simulation + MoveIt

```bash
ros2 launch cynlr_bringup cynlr_system.launch.py \
  vendor:=sim \
  use_moveit:=true \
  use_rviz:=true
```

MoveIt's `move_group` starts with OMPL planners, KDL IK, and the three planning groups.
Use RViz's MotionPlanning panel to plan and execute trajectories.

#### Real hardware (all three arms)

```bash
ros2 launch cynlr_bringup cynlr_system.launch.py \
  vendor:=flexiv \
  sn_left:=Rizon4s-062000x \
  sn_center:=Rizon4s-062001x \
  sn_right:=Rizon4s-062002x \
  use_moveit:=true \
  use_rviz:=true
```

#### Real hardware, left arm only (center + right as sim)

```bash
ros2 launch cynlr_bringup cynlr_system.launch.py \
  vendor:=flexiv \
  sn_left:=Rizon4s-062000x \
  sn_center:="" \
  sn_right:="" \
  use_rviz:=true
```

> Note: `vendor:=flexiv` applies to all three instances. Pass empty serial numbers for arms
> not yet available — the hardware plugin will fail to connect and enter error state. A
> per-arm vendor param is a future improvement.

### Controller YAML tuning

Key parameters in `cynlr_bringup/config/cynlr_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000           # Hz — all three arms, all controllers

arm_left_state_broadcaster:
  ros__parameters:
    publish_rate: 100.0         # Hz for ArmState; FT/TCP always at 1000 Hz

arm_left_jt_controller:
  ros__parameters:
    constraints:
      stopped_velocity_tolerance: 0.01   # rad/s — how close to "stopped"
      goal_time: 5.0                     # s — allowed overtime on trajectory
```

### Tool configuration

After startup, set the tool payload so FT readings and gravity compensation are correct:

```bash
ros2 service call /arm_service_left/set_tool cynlr_arm_interfaces/srv/SetTool \
  '{mass_kg: 0.8, com: [0.0, 0.0, 0.05], inertia: [0.001, 0.001, 0.001, 0.0, 0.0, 0.0],
    tcp_pose: [0.0, 0.0, 0.12, 1.0, 0.0, 0.0, 0.0]}'
```

Zero the FT sensor after mounting the tool:

```bash
ros2 service call /arm_service_left/zero_ft_sensor cynlr_arm_interfaces/srv/Trigger
```

### Camera calibration

Set calibration URL in `cynlr_camera/config/camera_config.yaml`:

```yaml
stereo_camera_node:
  ros__parameters:
    camera_index: 0
    publish_rate: 30.0
    calibration_url: "file:///home/cynlr/calibration/camera_0_left.yaml"
```

For OctoMap collision avoidance, the point cloud at `/camera_0/points` is consumed
automatically when MoveIt is running (configured in `sensors_3d.yaml`).

---

## Build and run

### Prerequisites

- Ubuntu 24.04 + ROS2 Jazzy (in WSL2 or native Linux)
- Conan 2.x (`pip install conan`)
- `flexiv_description` cloned into the colcon workspace

### Build cynlr_arm_core (Conan)

```bash
cd cynlr_arm_core
conan create . --profile=linux-gcc -s build_type=Release --build=missing

# Install CMake config so colcon can find it
conan install --requires=cynlr_arm_core/0.1.0 --profile=linux-gcc -s build_type=Release \
    -g CMakeDeps --output-folder=/opt/cynlr_cmake
```

### Build ROS2 packages (colcon)

```bash
# In workspace root (Cpp_App_Test)
source /opt/ros/jazzy/setup.bash

colcon build \
    --packages-select \
        cynlr_arm_interfaces \
        cynlr_arm_service \
        cynlr_hardware \
        cynlr_arm_description \
        cynlr_arm_controllers \
        cynlr_moveit_config \
        cynlr_camera \
        cynlr_bringup \
    --cmake-args \
        -DCMAKE_PREFIX_PATH=/opt/cynlr_cmake \
        -DCMAKE_BUILD_TYPE=Release
```

### Run

```bash
source install/setup.bash

# Simulation
ros2 launch cynlr_bringup cynlr_system.launch.py vendor:=sim use_rviz:=true

# Simulation + MoveIt
ros2 launch cynlr_bringup cynlr_system.launch.py vendor:=sim use_moveit:=true use_rviz:=true

# Camera node (separate terminal)
ros2 launch cynlr_camera stereo_camera.launch.py camera_index:=0
```

### Verify the system is up

```bash
# All three arms publishing state
ros2 topic hz /arm_left_arm_state   # → ~100 Hz
ros2 topic hz /arm_left_tcp_pose    # → ~1000 Hz

# All controllers loaded
ros2 control list_controllers

# Expected: 13 active + 9 inactive = 22 total
# active:   joint_state_broadcaster
#           arm_{left,center,right}_state_broadcaster
#           arm_{left,center,right}_jt_controller
# inactive: arm_{left,center,right}_{direct_cmd,nrt,cartesian}
```
