# Robot Arm Service — Design Specification

**Date:** 2026-04-13
**Status:** Draft
**Scope:** cynlr_arm_core, cynlr_arm_interfaces, cynlr_arm_service

---

## 1. Overview

Standalone C++ robot arm service replacing LabVIEW state machines. One process per hardware instance. Layered architecture: vendor-agnostic core library, ROS2 service node, and message definitions as separate packages.

Part of a larger migration from LabVIEW to a full C++ robotics stack with ROS2/DDS communication. The orchestrator (BT engine + controllers) is a separate sub-project and out of scope for this spec.

### Goals

- One process per arm instance, spawn N arms = N services
- Vendor-agnostic core — Flexiv today, other robots tomorrow
- Cross-platform — Linux (Ubuntu 24.04) + Windows (MSVC)
- Two command paths: RT streaming (1kHz) and NRT discrete commands
- Simulation backend swappable via config, zero code change
- LabVIEW interop via DDS (RTI DDS Toolkit), no bridge process

### Non-Goals

- Orchestrator / BT engine design (separate spec)
- IK solvers (separate library)
- Gripper / camera / motor services (future specs, same pattern)
- ros2_control integration (controller logic lives in orchestrator, not arm service)

---

## 2. Package Architecture

### Dependency Graph

```
cynlr_arm_core            (pure C++ library, no ROS2)
├── flexiv_rdk, spdlog, tl::expected
│
cynlr_arm_interfaces      (ROS2 package, messages only)
├── std_msgs, geometry_msgs, sensor_msgs
│
cynlr_arm_service         (ROS2 C++ node)
├── cynlr_arm_core, cynlr_arm_interfaces
├── rclcpp, rclcpp_lifecycle, rclcpp_action
```

### Build Order

1. `cynlr_arm_core` — Conan package, standard CMake
2. `cynlr_arm_interfaces` — colcon, message generation
3. `cynlr_arm_service` — colcon, depends on 1 + 2

---

## 3. cynlr_arm_core — Core Library

### 3.1 Directory Structure

```
cynlr_arm_core/
├── include/cynlr_arm_core/
│   ├── arm_interface.hpp
│   ├── arm_factory.hpp
│   ├── types.hpp
│   ├── error.hpp
│   ├── arm_config.hpp
│   └── capabilities/
│       ├── force_controllable.hpp
│       ├── impedance_configurable.hpp
│       ├── null_space_configurable.hpp
│       └── digital_io_controllable.hpp
├── src/
│   ├── arm_factory.cpp
│   ├── flexiv/
│   │   ├── flexiv_arm.hpp
│   │   └── flexiv_arm.cpp
│   └── sim/
│       ├── sim_arm.hpp
│       └── sim_arm.cpp
├── conanfile.py
└── CMakeLists.txt
```

### 3.2 Abstract Base Class

```cpp
class ArmInterface {
public:
    virtual ~ArmInterface() = default;

    // Lifecycle
    virtual Expected<void> connect(const ArmConfig& config) = 0;
    virtual Expected<void> disconnect() = 0;
    virtual Expected<void> enable() = 0;
    virtual Expected<void> stop() = 0;

    // State
    virtual Expected<ArmState> get_state() = 0;
    virtual bool is_connected() const = 0;
    virtual bool has_fault() const = 0;
    virtual Expected<void> clear_fault() = 0;
    virtual Expected<void> run_auto_recovery() = 0;

    // Discrete motion (non-blocking)
    virtual Expected<void> move_l(const CartesianTarget& target, const MotionParams& params) = 0;
    virtual Expected<void> move_j(const JointTarget& target, const MotionParams& params) = 0;
    virtual Expected<void> move_ptp(const CartesianTarget& target, const MotionParams& params) = 0;
    virtual Expected<bool> is_motion_complete() = 0;

    // Streaming (1kHz)
    virtual Expected<void> start_streaming(StreamMode mode) = 0;
    virtual Expected<void> stream_command(const StreamCommand& cmd) = 0;
    virtual Expected<void> stop_streaming() = 0;

    // Tool
    virtual Expected<void> set_tool(const ToolInfo& tool) = 0;
    virtual Expected<void> zero_ft_sensor() = 0;

    // Capabilities
    virtual std::vector<std::string> supported_features() const = 0;
};
```

### 3.3 Capability Interfaces

Mix-in interfaces for vendor-specific features. Consumer checks via `dynamic_cast` or `supported_features()`.

```cpp
class ForceControllable {
public:
    virtual ~ForceControllable() = default;
    virtual Expected<void> set_force_control_axis(const ForceAxisConfig& config) = 0;
    virtual Expected<void> set_force_control_frame(const CartesianPose& frame) = 0;
    virtual Expected<void> set_passive_force_control(bool enable) = 0;
};

class ImpedanceConfigurable {
public:
    virtual ~ImpedanceConfigurable() = default;
    virtual Expected<void> set_cartesian_impedance(const ImpedanceParams& params) = 0;
    virtual Expected<void> set_joint_impedance(const JointImpedanceParams& params) = 0;
};

class NullSpaceConfigurable {
public:
    virtual ~NullSpaceConfigurable() = default;
    virtual Expected<void> set_null_space_posture(const JointState& posture) = 0;
    virtual Expected<void> set_null_space_objectives(const NullSpaceWeights& weights) = 0;
};

class DigitalIOControllable {
public:
    virtual ~DigitalIOControllable() = default;
    virtual Expected<void> set_digital_outputs(const std::vector<std::pair<int, bool>>& outputs) = 0;
    virtual Expected<std::vector<bool>> get_digital_inputs() = 0;
};
```

### 3.4 Vendor Implementations

**FlexivArm:**

```cpp
class FlexivArm : public ArmInterface,
                   public ForceControllable,
                   public ImpedanceConfigurable,
                   public NullSpaceConfigurable,
                   public DigitalIOControllable {
    // Wraps flexiv::rdk::Robot directly
    // Single robot instance per FlexivArm — no global state
};
```

Flexiv RDK features to expose:
- NRT: `SendJointPosition()`, `SendCartesianMotionForce()`
- RT: `StreamJointPosition()`, `StreamJointTorque()`, `StreamCartesianMotionForce()` (Linux)
- Force control: per-axis selection, passive/active, impedance
- Null space: posture + objectives
- Digital I/O: 18 in, 18 out
- Plan/primitive execution
- Auto recovery, mode switching

**SimArm:**

```cpp
class SimArm : public ArmInterface,
               public ForceControllable,
               public NullSpaceConfigurable {
    // Joint integrator + basic dynamics
    // Configurable: sensor noise, latency, fault injection
    // FK computed internally from joint state
};
```

### 3.5 Factory

```cpp
std::unique_ptr<ArmInterface> create_arm(const ArmConfig& config);
// config.vendor == "flexiv" → FlexivArm
// config.vendor == "sim"    → SimArm
```

### 3.6 Key Types

```cpp
struct ArmConfig {
    std::string vendor;          // "flexiv", "sim"
    std::string serial_number;
    std::string ip_address;
    int num_joints;
    // vendor-specific params as key-value map
    std::unordered_map<std::string, std::string> params;
};

struct ArmState {
    std::array<double, 7> joint_positions;       // rad
    std::array<double, 7> joint_velocities;      // rad/s
    std::array<double, 7> joint_torques;         // Nm
    std::array<double, 7> joint_torques_external; // Nm
    std::array<double, 7> tcp_pose;              // [x,y,z,qw,qx,qy,qz] m/quat
    std::array<double, 6> tcp_velocity;          // [vx,vy,vz,wx,wy,wz] m/s, rad/s
    std::array<double, 6> ft_sensor_raw;         // [fx,fy,fz,tx,ty,tz] N/Nm
    std::array<double, 6> ext_wrench_in_tcp;     // N/Nm
    std::array<double, 6> ext_wrench_in_world;   // N/Nm
    bool fault;
    bool operational;
    bool estopped;
};

struct CartesianTarget {
    std::array<double, 7> pose;  // [x,y,z,qw,qx,qy,qz] m/quat
};

struct JointTarget {
    std::array<double, 7> positions;  // rad
};

struct MotionParams {
    double max_linear_vel;       // m/s
    double max_angular_vel;      // rad/s
    double max_linear_acc;       // m/s^2
    double max_joint_vel;        // rad/s
    double max_joint_acc;        // rad/s^2
};

enum class StreamMode {
    JOINT_POSITION,
    JOINT_TORQUE,
    CARTESIAN_MOTION_FORCE
};

struct StreamCommand {
    StreamMode mode;
    std::array<double, 7> joint_position;     // rad
    std::array<double, 7> joint_velocity;     // rad/s
    std::array<double, 7> joint_acceleration; // rad/s^2
    std::array<double, 7> joint_torque;       // Nm
    std::array<double, 7> cartesian_pose;     // m/quat
    std::array<double, 6> wrench;             // N/Nm
};
```

Internal units: SI (metres, radians, Nm). No degree/mm conversions inside core.

---

## 4. cynlr_arm_interfaces — Message Definitions

### 4.1 Messages

**ArmState.msg:**
```
std_msgs/Header header
float64[7] joint_positions
float64[7] joint_velocities
float64[7] joint_torques
float64[7] joint_torques_external
geometry_msgs/Pose tcp_pose
geometry_msgs/Twist tcp_velocity
geometry_msgs/Wrench ft_sensor_raw
geometry_msgs/Wrench ext_wrench_in_tcp
geometry_msgs/Wrench ext_wrench_in_world
bool fault
bool operational
bool estopped
```

**JointCommand.msg:**
```
std_msgs/Header header
float64[7] position
float64[7] velocity
float64[7] acceleration
float64[7] torque
uint8 mode  # 0=JOINT_POSITION, 1=JOINT_TORQUE
```

**CartesianCommand.msg:**
```
std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Wrench wrench
geometry_msgs/Twist velocity
geometry_msgs/Accel acceleration
```

### 4.2 Services

**Connect.srv:**
```
string config_yaml  # YAML string or path
---
bool success
string message
```

**SetMode.srv:**
```
uint8 NRT_MODE=0
uint8 RT_JOINT_POSITION=1
uint8 RT_JOINT_TORQUE=2
uint8 RT_CARTESIAN_MOTION_FORCE=3
uint8 mode
---
bool success
string message
```

**SetTool.srv:**
```
float64 mass_kg
float64[3] com       # center of mass [x,y,z] m
float64[6] inertia   # kg*m^2
float64[7] tcp_pose  # [x,y,z,qw,qx,qy,qz]
---
bool success
string message
```

**Trigger.srv** (for enable, disable, stop, clear_fault, zero_ft_sensor, auto_recovery):
```
---
bool success
string message
```

**SetDigitalOutputs.srv:**
```
uint8[] ports
bool[] values
---
bool success
string message
```

**GetDigitalInputs.srv:**
```
---
bool[18] values
```

### 4.3 Actions

**MoveL.action:**
```
# Goal
geometry_msgs/Pose target
float64 max_linear_vel      # m/s
float64 max_angular_vel     # rad/s
float64 max_linear_acc      # m/s^2
---
# Result
bool success
string message
---
# Feedback
geometry_msgs/Pose current_pose
float64 distance_remaining   # m
```

**MoveJ.action:**
```
# Goal
float64[7] target_joints     # rad
float64 max_joint_vel        # rad/s
float64 max_joint_acc        # rad/s^2
---
# Result
bool success
string message
---
# Feedback
float64[7] current_joints
float64 distance_remaining
```

**MovePTP.action:**
```
# Goal
geometry_msgs/Pose target
float64 joint_vel_scale      # 0.0 - 1.0
---
# Result
bool success
string message
---
# Feedback
geometry_msgs/Pose current_pose
float64 distance_remaining
```

---

## 5. cynlr_arm_service — ROS2 Node

### 5.1 Directory Structure

```
cynlr_arm_service/
├── include/cynlr_arm_service/
│   ├── arm_service_node.hpp
│   ├── rt_thread.hpp
│   └── mode_manager.hpp
├── src/
│   ├── arm_service_node.cpp
│   ├── rt_thread.cpp
│   ├── mode_manager.cpp
│   └── main.cpp
├── config/
│   └── arm_config.yaml
├── launch/
│   └── arm_service.launch.py
├── CMakeLists.txt
└── package.xml
```

### 5.2 Node Architecture

```cpp
class ArmServiceNode : public rclcpp_lifecycle::LifecycleNode {
    std::unique_ptr<cynlr_arm_core::ArmInterface> arm_;
    std::unique_ptr<RTThread> rt_thread_;
    std::unique_ptr<ModeManager> mode_manager_;

    // RT Path
    rclcpp::Subscription<JointCommand>::SharedPtr sub_joint_command_;
    rclcpp::Subscription<CartesianCommand>::SharedPtr sub_cartesian_command_;
    rclcpp::Publisher<ArmState>::SharedPtr pub_arm_state_;

    // NRT Services
    rclcpp::Service<Connect>::SharedPtr srv_connect_;
    rclcpp::Service<Trigger>::SharedPtr srv_enable_;
    rclcpp::Service<Trigger>::SharedPtr srv_stop_;
    rclcpp::Service<Trigger>::SharedPtr srv_clear_fault_;
    rclcpp::Service<SetTool>::SharedPtr srv_set_tool_;
    rclcpp::Service<Trigger>::SharedPtr srv_zero_ft_sensor_;
    rclcpp::Service<Trigger>::SharedPtr srv_auto_recovery_;
    rclcpp::Service<SetMode>::SharedPtr srv_set_mode_;
    rclcpp::Service<SetDigitalOutputs>::SharedPtr srv_set_digital_outputs_;
    rclcpp::Service<GetDigitalInputs>::SharedPtr srv_get_digital_inputs_;

    // NRT Actions
    rclcpp_action::Server<MoveL>::SharedPtr act_move_l_;
    rclcpp_action::Server<MoveJ>::SharedPtr act_move_j_;
    rclcpp_action::Server<MovePTP>::SharedPtr act_move_ptp_;

    // Event Publishers
    rclcpp::Publisher<FaultEvent>::SharedPtr pub_fault_event_;
    rclcpp::Publisher<OperationalStatus>::SharedPtr pub_operational_status_;
};
```

### 5.3 RT Thread

Dedicated thread for 1kHz command streaming and state publishing.

```cpp
class RTThread {
    std::thread thread_;
    std::atomic<bool> running_;

    // Lock-free triple buffer for commands and state
    LockFreeBuffer<StreamCommand> command_buffer_;
    LockFreeBuffer<ArmState> state_buffer_;

    cynlr_arm_core::ArmInterface* arm_;  // non-owning

    void run() {
        configure_rt_scheduling();  // SCHED_FIFO on Linux, TIME_CRITICAL on Windows
        while (running_) {
            auto cmd = command_buffer_.load();
            arm_->stream_command(cmd);
            auto state = arm_->get_state();
            state_buffer_.store(state);
            wait_until_next_period(cycle_time_);
        }
    }
};
```

RT configuration via launch params:

```yaml
rt_config:
  cycle_time_us: 1000
  cpu_affinity: -1
  scheduler: "fifo"
  priority: 90
  command_timeout_ms: 100
```

### 5.4 Mode Manager

State machine governing allowed operations:

```
IDLE → (connect) → CONNECTED → (enable) → ENABLED
ENABLED → (set_mode NRT) → NRT_MODE
ENABLED → (set_mode RT)  → RT_MODE
NRT_MODE / RT_MODE → (stop) → ENABLED
Any → (fault) → FAULT
FAULT → (clear_fault) → ENABLED (if clearable)
Any → (disconnect) → IDLE
```

Mode manager enforces:
- Actions (moveL/J/PTP) only in NRT_MODE
- RT topic streaming only in RT_MODE
- Service calls (set_tool, zero_ft) only in ENABLED or above
- Connect/disconnect always allowed

### 5.5 Namespace and Configuration

Logical namespace as ROS2 namespace. Serial number in config.

```yaml
# arm_config.yaml
vendor: "flexiv"
serial_number: "Rizon4s-ABC123"
ip_address: "192.168.2.100"
num_joints: 7
state_publish_rate: 500.0    # Hz, configurable
params:
  # vendor-specific key-value pairs
```

Launch:

```bash
ros2 launch cynlr_arm_service arm_service.launch.py \
    namespace:=left_arm \
    config:=left_arm.yaml
```

Simulation — change config only:

```yaml
vendor: "sim"
num_joints: 7
state_publish_rate: 500.0
params:
  noise_stddev: "0.001"
  latency_ms: "0.5"
```

---

## 6. QoS Configuration

| Topic | Reliability | Durability | History | Depth |
|-------|------------|-----------|---------|-------|
| `cmd/joint` | BEST_EFFORT | VOLATILE | KEEP_LAST | 1 |
| `cmd/cartesian` | BEST_EFFORT | VOLATILE | KEEP_LAST | 1 |
| `state` | BEST_EFFORT | VOLATILE | KEEP_LAST | 1 |
| `fault_event` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST | 10 |
| `operational_status` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST | 1 |

Rationale:
- RT command/state topics: latest value matters, skip stale, minimize latency
- Fault/status: never miss, late joiners see current state

---

## 7. Testing Strategy

### Unit Tests (cynlr_arm_core)

- SimArm as test double, no hardware
- All ArmInterface methods
- Capability interface detection
- Factory creates correct vendor
- Mode state machine transitions
- Framework: Google Test

### Integration Tests (cynlr_arm_service)

- Launch with SimArm backend
- ROS2 service calls → verify responses
- Action goals → verify feedback + completion
- RT command publish → verify state response
- Mode switching (NRT ↔ RT)
- Fault injection + recovery flow
- Framework: launch_testing + Google Test

### System Tests (hardware)

- Manual / CI-optional with real Flexiv arm
- Verify motion commands execute correctly
- RT streaming latency meets 1ms requirement
- Linux only (RT streaming)

---

## 8. Error Handling

- All core methods return `tl::expected<T, Error>` — no exceptions across boundaries
- Error struct contains error code + human-readable message
- Arm service translates errors to ROS2 service/action responses
- Faults published on `fault_event` topic — orchestrator handles recovery
- RT thread: if no command received within `command_timeout_ms`, arm stops streaming

---

## 9. Future Extensions

Following same layered pattern:
- `cynlr_gripper_core` + `cynlr_gripper_service`
- `cynlr_camera_core` + `cynlr_camera_service`
- `cynlr_motor_core` + `cynlr_motor_service`
- `cynlr_orchestrator` — BT engine, controller hosting, LabVIEW interface (separate spec)

Each hardware type gets: core library (Conan) + interfaces package (colcon) + service node (colcon).
