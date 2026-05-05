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
5. [cynlr_robot](#cynlr_robot)
6. [cynlr_arm_node](#cynlr_arm_node)
7. [cynlr_arm_description](#cynlr_arm_description)
8. [cynlr_arm_controllers](#cynlr_arm_controllers)
9. [cynlr_moveit_config](#cynlr_moveit_config)
10. [cynlr_camera](#cynlr_camera)
11. [cynlr_bringup](#cynlr_bringup)
12. [How the packages connect](#how-the-packages-connect)
13. [The three motion modes](#the-three-motion-modes)
14. [Switching modes at runtime](#switching-modes-at-runtime)
15. [Configuration reference](#configuration-reference)
16. [Build and run](#build-and-run)

---

## Layer diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  Your code / AI policy / BT engine                                          │
│  publishes ROS topics, calls ROS services, sends action goals               │
└──────────┬────────────────────────┬────────────────────────┬────────────────┘
           │                        │                        │
    ┌──────▼──────┐          ┌──────▼──────┐         ┌──────▼──────────────┐
    │   MoveIt    │          │ CynlrArmNode│         │   cynlr_camera      │
    │  move_group │          │  per arm    │         │   stereo pair       │
    │  (optional) │          │  (NRT ops + │         │                     │
    └──────┬──────┘          │   state pub)│         └─────────────────────┘
           │ FollowJoint     └──────┬──────┘
           │ Trajectory             │ CynlrArmHandle (via CynlrArmRegistry)
           │                        │
┌──────────▼────────────────────────▼────────────────────────────────────────┐
│  cynlr_main                                                                 │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │  ControllerManager  (cm_update_thread: 500 Hz read/update/write)      │ │
│  │                                                                       │ │
│  │  ┌──────────────────┐  ┌──────────────────┐  ┌────────────────────┐  │ │
│  │  │ JointTrajCtrl    │  │ DirectCmdCtrl    │  │  CartesianCtrl     │  │ │
│  │  │ (default/MoveIt) │  │ (escape hatch #1)│  │  (escape hatch #2) │  │ │
│  │  └──────────────────┘  └──────────────────┘  └────────────────────┘  │ │
│  │  ┌───────────────────────────────────────────────────────────────┐   │ │
│  │  │  joint_state_broadcaster  (standard ROS2 controller)          │   │ │
│  │  └───────────────────────────────────────────────────────────────┘   │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │  CynlrArmNode × N  (arm_thread executor, one per arm)                 │ │
│  │  Polls CynlrArmRegistry, then:                                        │ │
│  │    publishes: tcp_pose, ft_raw, ext_wrench, arm_state (100 Hz)        │ │
│  │    services:  clear_fault, set_tool, zero_ft_sensor                   │ │
│  │    actions:   move_l, move_j, move_ptp (NRT, via CynlrArmHandle)      │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────┬──────────────────────────────────────────┘
                                  │ StateInterface / CommandInterface
┌─────────────────────────────────▼──────────────────────────────────────────┐
│  cynlr_robot  CynlrRobotInterface  (one instance per arm)                   │
│  on_activate: connect → enable → set_tool → register CynlrArmHandle        │
│  read:  get_state() → fill hw_pos_/vel_/eff_, arm_state_                   │
│  write: dispatch by active_mode_ → stream_command / gpio                   │
└─────────────────────────────────┬──────────────────────────────────────────┘
                                  │  C++ API (no ROS)
┌─────────────────────────────────▼──────────────────────────────────────────┐
│  cynlr_arm_core   ArmInterface   (vendor-agnostic, pure C++20)              │
│  FlexivArm → Flexiv RDK        SimArm → integrator / FK                    │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## Package overview

| Package | Type | Purpose |
|---|---|---|
| `cynlr_arm_core` | CMake (C++, no ROS) | Vendor-agnostic arm API + implementations |
| `cynlr_arm_interfaces` | ROS2 ament | ROS message/service/action definitions |
| `cynlr_robot` | ROS2 ament, pluginlib | ros2_control hardware plugin + CynlrArmRegistry |
| `cynlr_arm_node` | ROS2 ament | Per-arm node: state publishing, NRT services, NRT actions |
| `cynlr_arm_description` | ROS2 ament | URDF/xacro for the multi-arm system |
| `cynlr_arm_controllers` | ROS2 ament, pluginlib | 2 RT controller plugins: direct_cmd, cartesian |
| `cynlr_moveit_config` | ROS2 ament | MoveIt SRDF, kinematics, planner config |
| `cynlr_camera` | ROS2 ament | Stereo camera lifecycle node |
| `cynlr_bringup` | ROS2 ament | Launch files + config YAML |

---

## cynlr_arm_core

**Language:** Pure C++20, no ROS, built with CMake (installed to `~/cynlr_software/cynlr_install/`).
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
```

### Capability interfaces

Optional mixins. Dynamic-cast the `ArmInterface*` to check support:

| Interface | Methods |
|---|---|
| `DigitalIOControllable` | `set_digital_outputs` / `get_digital_inputs` |
| `ForceControllable` | `set_force_control_axis` / `set_force_control_frame` / `set_passive_force_control` |
| `ImpedanceConfigurable` | `set_cartesian_impedance` / `set_joint_impedance` |
| `NullSpaceConfigurable` | `set_null_space_posture` / `set_null_space_objectives` |

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

**`FlexivArm`** — wraps `flexiv::rdk::Robot`. Implements all capability interfaces.
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
| `ArmState` | All fields from `cynlr::arm::ArmState` + header | `CynlrArmNode` |
| `JointCommand` | mode, joint_position[7], joint_velocity[7], joint_torque[7] | Your code → `CynlrDirectCommandController` |
| `CartesianCommand` | cartesian_pose[7], wrench[6] | Your code → `CynlrCartesianController` |

### Services

| Service | Handled by |
|---|---|
| `Trigger` | `CynlrArmNode`: clear_fault / zero_ft_sensor |
| `SetTool` | `CynlrArmNode`: set_tool |

### Actions

| Action | Hosted by |
|---|---|
| `MoveL` | `CynlrArmNode` — Cartesian linear move via arm's NRT planner |
| `MoveJ` | `CynlrArmNode` — joint-space move via arm's NRT planner |
| `MovePTP` | `CynlrArmNode` — Cartesian PTP move via arm's NRT planner |

---

## cynlr_robot

**Language:** C++20, pluginlib plugin.
**Location:** `cynlr_robot/`
**Plugin class:** `cynlr_robot/CynlrRobotInterface`

This package contains three components:

### CynlrRobotInterface

The bridge between ros2_control's update loop and `cynlr_arm_core`. One instance per arm,
all running inside the same `cynlr_main` process.

**`on_init()`:** reads `prefix`, `vendor`, `serial_number`, `ip_address`, `tool.*` from
URDF `<hardware>` params. Calls `create_arm(config)`. NaN-fills all command buffers.

**`on_activate()`:** `arm_->connect()` → `arm_->clear_fault()` → `arm_->enable()` →
`arm_->set_tool(tool_info_)` → registers `CynlrArmHandle` in `CynlrArmRegistry`. Seeds
`cmd_pos_` from current state so the arm holds still on first write.

**`on_deactivate()`:** unregisters from `CynlrArmRegistry` → `arm_->stop_streaming()` →
`arm_->stop()` → `arm_->disconnect()`.

**`read()`:** `arm_->get_state()` → copies into `hw_pos_`, `hw_vel_`, `hw_eff_`, `arm_state_`.

**`write()`:** dispatches by `active_mode_`:
- `POSITION` → `stream_command({JOINT_POSITION, cmd_pos_, cmd_vel_})`
- `EFFORT` → `stream_command({JOINT_TORQUE, cmd_eff_})`
- `CARTESIAN` → `stream_command({CARTESIAN_MOTION_FORCE, cmd_cart_[0..6], cmd_cart_[7..12]})`
- Skips send if any commanded value is `NaN`. GPIO outputs only written on diff.

### StateInterfaces exported (per arm)

| Name | Content |
|---|---|
| `{prefix}joint{1-7}/position` | joint angle (rad) |
| `{prefix}joint{1-7}/velocity` | joint velocity (rad/s) |
| `{prefix}joint{1-7}/effort` | joint torque (Nm) |
| `{prefix}ft_sensor/raw_{0-5}` | raw FT sensor [fx..tz] |
| `{prefix}ft_sensor/ext_tcp_{0-5}` | external wrench in TCP |
| `{prefix}ft_sensor/ext_world_{0-5}` | external wrench in world |
| `{prefix}tcp/pose_{0-6}` | TCP pose [x,y,z,qw,qx,qy,qz] |
| `{prefix}tcp/vel_{0-5}` | TCP velocity |
| `{prefix}gpio/in_{0-17}` | digital inputs |

### CommandInterfaces exported (per arm)

| Name | Written by |
|---|---|
| `{prefix}joint{1-7}/position` | JTC / DirectCmdCtrl |
| `{prefix}joint{1-7}/velocity` | DirectCmdCtrl |
| `{prefix}joint{1-7}/effort` | (future torque controller) |
| `{prefix}cartesian_cmd/pose_{0-6}` | CynlrCartesianController |
| `{prefix}cartesian_cmd/wrench_{0-5}` | CynlrCartesianController |
| `{prefix}gpio/out_{0-17}` | (future GPIO controller) |

### CynlrArmHandle

Facade struct exposed by `CynlrRobotInterface` to `CynlrArmNode` via the registry. All
fields are `std::function` wrappers around the underlying `ArmInterface` methods:

```cpp
struct CynlrArmHandle {
    function<optional<ArmState>()>              get_state;
    function<Expected<void>()>                  clear_fault;
    function<Expected<void>(ToolInfo)>          set_tool;
    function<Expected<void>()>                  zero_ft_sensor;
    function<Expected<void>(CartesianTarget, MotionParams)>  move_l;
    function<Expected<void>(JointTarget, MotionParams)>      move_j;
    function<Expected<void>(CartesianTarget, MotionParams)>  move_ptp;
    function<optional<bool>()>                  is_motion_complete;
    function<Expected<void>()>                  stop;
};
```

### CynlrArmRegistry

Process-local singleton. Maps arm prefix → `shared_ptr<CynlrArmHandle>`. Thread-safe.

```cpp
CynlrArmRegistry::instance().register_arm(prefix, handle);   // called by on_activate
CynlrArmRegistry::instance().unregister_arm(prefix);         // called by on_deactivate
CynlrArmRegistry::instance().get(prefix);                    // called by CynlrArmNode
```

`CynlrArmNode` polls `get()` every 100 ms until the handle becomes available, then sets up
publishers, services, and action servers. This handles the natural startup race between the
hardware plugin activating and the node initializing.

---

## cynlr_arm_node

**Language:** C++20.
**Location:** `cynlr_arm_node/`

Two components: `CynlrArmNode` (library) and `cynlr_main` (executable).

### CynlrArmNode

Plain `rclcpp::Node` — one instance per arm, running in `cynlr_main`'s `arm_thread` executor.

On construction, starts a 100 ms poll timer calling `CynlrArmRegistry::get(prefix_)`. Once
the handle is available, sets up all publishers, services, and action servers.

**Publishers (100 Hz via `publish_timer_`):**

| Topic | Type |
|---|---|
| `/{prefix}arm_state` | `cynlr_arm_interfaces/msg/ArmState` |
| `/{prefix}tcp_pose` | `geometry_msgs/msg/PoseStamped` |
| `/{prefix}ft_raw` | `geometry_msgs/msg/WrenchStamped` |
| `/{prefix}ext_wrench_tcp` | `geometry_msgs/msg/WrenchStamped` |
| `/{prefix}ext_wrench_world` | `geometry_msgs/msg/WrenchStamped` |

**Services:**

| Service | Type |
|---|---|
| `/{prefix}clear_fault` | `cynlr_arm_interfaces/srv/Trigger` |
| `/{prefix}set_tool` | `cynlr_arm_interfaces/srv/SetTool` |
| `/{prefix}zero_ft_sensor` | `cynlr_arm_interfaces/srv/Trigger` |

**Action servers:**

| Action | Type |
|---|---|
| `/{prefix}move_l` | `cynlr_arm_interfaces/action/MoveL` |
| `/{prefix}move_j` | `cynlr_arm_interfaces/action/MoveJ` |
| `/{prefix}move_ptp` | `cynlr_arm_interfaces/action/MovePTP` |

NRT action goals execute in a detached thread. Each action polls `is_motion_complete()` and
publishes feedback while waiting. An `atomic<bool> motion_running_` flag prevents concurrent
goals.

### cynlr_main

Container executable that runs `ControllerManager` and N `CynlrArmNode` instances in a
single process, sharing the `CynlrArmRegistry` singleton.

```
Usage: cynlr_main <prefix1> [prefix2 ...] [--ros-args ...]
```

**Three threads:**

| Thread | Role |
|---|---|
| `cm_executor` (main) | Handles all ROS CM callbacks (services, spawner requests) |
| `cm_update_thread` | `500 Hz`: `cm->read()`, `cm->update()`, `cm->write()` |
| `arm_thread` | Spins `arm_executor` for all `CynlrArmNode` instances |

Keeping `cm_update_thread` separate from `cm_executor` is essential: `switch_controllers`
blocks the CM service callback waiting for `update()` to process the switch — both on the
same executor would deadlock.

---

## cynlr_arm_description

**Language:** xacro (XML macro).
**Location:** `cynlr_arm_description/`

**`urdf/cynlr_rizon7_macro.urdf.xacro`** — macro for a single arm.

Parameters: `prefix`, `serial_number`, `vendor`, `parent`, `*origin`, `tool_*`

Includes `$(find flexiv_description)/urdf/rizon7_macro.urdf.xacro` for Flexiv's link
geometry and kinematics, then adds the CynLR `<ros2_control>` block with all interface
declarations and hardware parameters.

**URDF generation:** `cynlr_system.launch.py` generates the URDF string at launch time via
`xacro` using parameters read from `cynlr_system_config.yaml`. There is no static URDF file
in the install tree — the config YAML is the single source of truth.

**Dependency:** requires `flexiv_description` (humble branch) to be in the colcon workspace.

---

## cynlr_arm_controllers

**Language:** C++20, pluginlib.
**Location:** `cynlr_arm_controllers/`

Two RT controller plugins. Both use `realtime_tools::RealtimeBuffer` for NRT→RT handoff.

### CynlrDirectCommandController — Escape Hatch #1

**Plugin:** `cynlr_direct_command_controller/CynlrDirectCommandController`

Claims joint position + velocity command interfaces. Subscribes to `/{prefix}joint_cmd_direct`
(`JointCommand`) with BEST_EFFORT QoS. Seeds positions from current state on activate.

In `update()`: reads from `RealtimeBuffer` and copies `joint_position[7]` and
`joint_velocity[7]` directly to command interfaces.

**Use this when:** you have a custom planner, learning policy, or teleoperation node
publishing joint commands at up to the CM update rate.

### CynlrCartesianController — Escape Hatch #2

**Plugin:** `cynlr_cartesian_controller/CynlrCartesianController`

Claims 13 GPIO-type command interfaces (`cartesian_cmd/pose_0..6`, `cartesian_cmd/wrench_0..5`).
Subscribes to `/{prefix}cartesian_cmd` (`CartesianCommand`) with BEST_EFFORT QoS. NaN-fills
on deactivate.

Hardware `write()` detects `CARTESIAN` mode and calls `arm_->stream_command({CARTESIAN_MOTION_FORCE, ...})`.

**Use this when:** running force-controlled tasks, visual servoing, or Cartesian-space policies.

---

## cynlr_moveit_config

**Language:** xacro, YAML.
**Location:** `cynlr_moveit_config/`

Configuration files for MoveIt's `move_group` node. No compiled code.

### Planning groups (SRDF)

| Group | Chain |
|---|---|
| `arm_left_arm` | `arm_left_base_link` → `arm_left_flange` |
| `arm_center_arm` | `arm_center_base_link` → `arm_center_flange` |
| `arm_right_arm` | `arm_right_base_link` → `arm_right_flange` |
| `all_arms` | group-of-groups, coordinated 3-arm planning with collision avoidance |

### Config files

| File | Purpose |
|---|---|
| `srdf/cynlr_three_arm.srdf.xacro` | Groups, named states, collision exclusions |
| `config/kinematics.yaml` | KDL solver per arm (swap for custom IK plugin) |
| `config/moveit_controllers.yaml` | Maps groups → `arm_*_jt_controller` action servers |
| `config/ompl_planning.yaml` | OMPL planners |
| `config/sensors_3d.yaml` | OctoMap updater from `/camera_0/points` |

---

## cynlr_camera

**Language:** C++20, ROS2 lifecycle node.
**Location:** `cynlr_camera/`

`StereoCameraNode` publishes images, camera_info, and PointCloud2 per stereo pair. The
default implementation produces synthetic 1×1 frames so all topics exist without hardware.

To add real hardware: subclass and override `open_camera()`, `close_camera()`, `grab_frame()`.

**Topics (per camera index N):** `/camera_N/{left,right}/image_raw`, `/camera_N/{left,right}/camera_info`, `/camera_N/points`

---

## cynlr_bringup

**Language:** Python (launch), YAML.
**Location:** `cynlr_bringup/`

### cynlr_system_config.yaml

Single source of truth for the system. All launch behavior is driven from this file.

```yaml
system:
  update_rate: 500   # Hz

arms:
  - name: arm_left
    vendor: flexiv               # "flexiv" or "sim"
    serial_number: "Rizon4s-..."
    mount:
      xyz: [-0.5, 0.0, 0.9]
      rpy: [0.0, 0.0, 0.0]
    tool:
      mass_kg: 0.5
      com: [0.0, 0.0, 0.05]
      inertia: [0.001, 0.001, 0.001, 0.0, 0.0, 0.0]
      tcp_pose: [0.0, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0]

  - name: arm_center
    vendor: sim
    ...
```

Separate configs: `cynlr_single_arm_config.yaml` (one real arm) and `cynlr_system_config.yaml`
(three arms, left=real, center+right=sim).

### cynlr_system.launch.py

**Arguments:**

| Argument | Default | Description |
|---|---|---|
| `config_file` | `cynlr_system_config.yaml` path | Path to system config YAML |
| `use_moveit` | `false` | Launch `move_group` |
| `use_rviz` | `true` | Launch RViz |

The launch file reads `config_file` at launch time, generates the URDF string via xacro,
writes a temp controller YAML, then spawns `cynlr_main <prefix1> [prefix2 ...]`.

**Startup sequence:**

```
1. robot_state_publisher      (URDF from generated string)
2. cynlr_main                 (ControllerManager + CynlrArmNode × N)
3. joint_state_broadcaster    (spawner, waits for CM)
4. arm_*_jt_controller        (spawner, one per arm in sequence)
5. arm_*_direct_cmd           (spawner --inactive)
6. arm_*_cartesian            (spawner --inactive)
7. move_group + rviz2         (conditional)
```

---

## How the packages connect

Dependencies flow in one direction only — nothing below imports anything above.

```
cynlr_arm_core          ← no ROS, no upward dependencies
    ↑
cynlr_robot             ← links cynlr_arm_core; implements SystemInterface; exports CynlrArmRegistry
    ↑
cynlr_arm_description   ← URDF references cynlr_robot plugin by name (string in XML)
    ↑
cynlr_arm_controllers   ← claims StateInterfaces/CommandInterfaces from cynlr_robot
cynlr_arm_node          ← links cynlr_robot (for CynlrArmHandle/Registry); uses cynlr_arm_interfaces
cynlr_moveit_config     ← YAML only; references JTC action server names
cynlr_camera            ← standalone; no cynlr_arm_core dependency
cynlr_bringup           ← wires everything via launch + YAML
```

### Data flow at runtime (500 Hz update loop)

```
arm hardware
    │
    ▼  arm_->get_state()
CynlrRobotInterface::read()
    │  fills hw_pos_[], hw_vel_[], hw_eff_[], arm_state_
    │
    ▼  StateInterface slots
controller_manager distributes to controllers
    │
    ├─► JointTrajectoryController::update()       (if active)
    │      reads hw_pos_[], hw_vel_[] → PID → writes cmd_pos_[]
    │
    ├─► CynlrDirectCommandController::update()    (if active)
    │      reads RealtimeBuffer → writes cmd_pos_[], cmd_vel_[]
    │
    └─► CynlrCartesianController::update()        (if active)
           reads RealtimeBuffer → writes cmd_cart_[]
    │
    ▼  CommandInterface slots
CynlrRobotInterface::write()
    │
    ▼  arm_->stream_command(cmd)
arm hardware
```

### State publishing (arm_thread, 100 Hz)

```
CynlrRobotInterface::read()   [cm_update_thread]
    │  updates arm_state_ in place  (lock-free: written once per tick,
    │  CynlrArmHandle.get_state() copies it out under SDK's internal lock)
    │
CynlrArmNode::publish_state() [arm_thread, 100 Hz timer]
    │  calls handle_->get_state()
    ▼  publishes arm_state, tcp_pose, ft_raw, ext_wrench_*
```

### NRT motion flow

```
Action client
    │  goal: MoveJ {target_positions, max_joint_vel}
    ▼
CynlrArmNode (arm_thread)
    │  sets motion_running_ = true
    │  spawns detached thread:
    │      handle_->move_j(target, params)    ← NRT SDK call, returns immediately
    │      loop every 20ms:
    │          handle_->is_motion_complete()
    │          publish feedback
    ▼  goal result: succeed / abort
```

---

## The three motion modes

| Mode | Active controller | Planning | Use case |
|---|---|---|---|
| **MoveIt** | `arm_*_jt_controller` (JTC) | MoveIt OMPL | Multi-arm planning, collision avoidance, IK |
| **Direct joint** | `arm_*_direct_cmd` | Your code | Learning policies, custom planners, teleoperation |
| **Cartesian** | `arm_*_cartesian` | Your code | Force control, visual servoing, Cartesian policies |

**NRT motion** (`move_l/j/ptp` actions via `CynlrArmNode`) does not require switching
controllers — it is always available via the arm thread. The arm's internal planner generates
the trajectory.

Only one RT controller per arm can own the command interfaces at a time. Switch at runtime;
no restart required.

---

## Switching modes at runtime

### Default (MoveIt / JTC) → Direct joint commands

```bash
ros2 control switch_controllers \
  --deactivate arm_left_jt_controller \
  --activate   arm_left_direct_cmd --strict

ros2 topic pub /arm_left_joint_cmd_direct \
  cynlr_arm_interfaces/msg/JointCommand \
  '{mode: 0, joint_position: [0.0, -0.5, 0.0, 1.2, 0.0, 0.5, 0.0]}' --once
```

### Default → Cartesian force/motion control

```bash
ros2 control switch_controllers \
  --deactivate arm_left_jt_controller \
  --activate   arm_left_cartesian --strict

ros2 topic pub /arm_left_cartesian_cmd \
  cynlr_arm_interfaces/msg/CartesianCommand \
  '{cartesian_pose: [0.4, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0], wrench: [0.0, 0.0, -5.0, 0.0, 0.0, 0.0]}' --once
```

### Switch back to default

```bash
ros2 control switch_controllers \
  --deactivate arm_left_direct_cmd arm_left_cartesian \
  --activate   arm_left_jt_controller --strict
```

### NRT move (no controller switch needed)

```bash
ros2 action send_goal /arm_left_move_j cynlr_arm_interfaces/action/MoveJ \
  '{target_positions: [0.0, -0.5, 0.0, 1.2, 0.0, 0.5, 0.0], max_joint_vel: 0.5}'
```

---

## Configuration reference

### Single arm (one real Flexiv, no others)

```bash
ros2 launch cynlr_bringup cynlr_system.launch.py \
  config_file:=$(ros2 pkg prefix cynlr_bringup)/share/cynlr_bringup/config/cynlr_single_arm_config.yaml \
  use_rviz:=false use_moveit:=false
```

Expected controllers: `joint_state_broadcaster` [active], `arm_left_jt_controller` [active],
`arm_left_direct_cmd` [inactive], `arm_left_cartesian` [inactive].

### Three arms (left=real, center+right=sim)

```bash
ros2 launch cynlr_bringup cynlr_system.launch.py use_rviz:=false use_moveit:=false
# Uses default config_file: cynlr_system_config.yaml
```

Expected controllers: 10 total (1 JSB + 3×JTC active, 3×direct_cmd + 3×cartesian inactive).

> **WSL2 note:** Multi-arm JT controller activation may time out due to non-RT jitter from
> Flexiv's `stream_command` blocking the 500 Hz update thread. This is a WSL2 scheduler
> issue, not a code bug. On a real-time OS it is not present.

### Tool configuration

Tool params are set in `cynlr_system_config.yaml` under `arms[*].tool` and applied
automatically in `CynlrRobotInterface::on_activate()`. To update at runtime:

```bash
ros2 service call /arm_left_clear_fault cynlr_arm_interfaces/srv/Trigger {}
ros2 service call /arm_left_set_tool cynlr_arm_interfaces/srv/SetTool \
  '{mass_kg: 0.8, com: [0.0, 0.0, 0.05], inertia: [0.001, 0.001, 0.001, 0.0, 0.0, 0.0],
    tcp_pose: [0.0, 0.0, 0.12, 1.0, 0.0, 0.0, 0.0]}'
ros2 service call /arm_left_zero_ft_sensor cynlr_arm_interfaces/srv/Trigger {}
```

---

## Build and run

### Prerequisites

```bash
sudo apt install -y \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-realtime-tools ros-jazzy-hardware-interface \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller
```

### Build flexiv_rdk from submodule (one-time)

```bash
git submodule update --init --recursive

cmake -S flexiv_rdk -B flexiv_rdk/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/cynlr_software/rdk_install
cmake --build flexiv_rdk/build --config Release -j$(nproc)
cmake --install flexiv_rdk/build
```

### Build cynlr_arm_core (one-time)

```bash
cmake -S cynlr_arm_core -B cynlr_arm_core/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$HOME/cynlr_software/rdk_install" \
  -DCMAKE_INSTALL_PREFIX=$HOME/cynlr_software/cynlr_install
cmake --build cynlr_arm_core/build -j$(nproc)
cmake --install cynlr_arm_core/build
```

### Build all colcon packages

```bash
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

### Verify the system is up

```bash
ros2 control list_controllers
ros2 topic hz /joint_states              # ~500 Hz
ros2 topic echo /arm_left_arm_state --once
```
