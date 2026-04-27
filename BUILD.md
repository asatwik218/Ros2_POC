# Build & Test Guide

## Status (as of 2026-04-27)

| Package | Built | Tests |
|---|---|---|
| cynlr_arm_core | ✅ (Conan, installs to `~/cynlr_software/cynlr_install`) | ✅ unit tests pass |
| cynlr_arm_interfaces | ✅ colcon | — |
| cynlr_arm_service | ✅ colcon | ✅ 5/5 integration tests pass |
| cynlr_hardware | ✅ colcon | — |
| cynlr_arm_controllers | ✅ colcon | — |
| cynlr_camera | ✅ colcon | — |
| cynlr_arm_description | ✅ colcon | ✅ xacro parses cleanly |
| cynlr_moveit_config | ✅ colcon | — |
| cynlr_bringup | ✅ colcon | ✅ system launch: 16 controllers, 7 active, joint_states ~1000 Hz |
| flexiv_description | ✅ colcon (humble branch) | — |

**NEXT STEP: Escape hatch switching tests** — see section below.

---

## One-time prerequisites

```bash
# 1. ros2-control stack + xacro
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-realtime-tools \
  ros-jazzy-hardware-interface \
  ros-jazzy-camera-info-manager \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher

# 2. cynlr_arm_core is pre-built; already installed at:
#    ~/cynlr_software/cynlr_install/
#    (CMAKE_PREFIX_PATH must include this path when building colcon packages)
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
    cynlr_arm_service \
    cynlr_hardware \
    cynlr_arm_controllers \
    cynlr_camera \
    cynlr_arm_description \
    cynlr_moveit_config \
    cynlr_bringup \
  --cmake-args \
    -DCMAKE_PREFIX_PATH="$HOME/cynlr_software/cynlr_install;/opt/ros/jazzy"

source install/setup.bash
```

---

## Run integration tests (cynlr_arm_service)

```bash
cd ~/cynlr_software/Cpp_App_Test
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon test --packages-select cynlr_arm_service --event-handlers console_direct+
colcon test-result --all --verbose
# Expected: 5/5 PASS
```

---

## System smoke test

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch cynlr_bringup cynlr_system.launch.py \
  vendor:=sim sn_left:=sim0 sn_center:=sim1 sn_right:=sim2 \
  use_rviz:=false use_moveit:=false
```

In a second terminal:
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 control list_controllers          # 16 controllers: 7 active, 9 inactive
ros2 topic echo /arm_left_arm_state --once  # ArmState messages
ros2 topic hz /joint_states                 # ~1000 Hz
```

---

## NEXT STEP: Escape hatch switching

With the system launch running, switch controllers and test each hatch:

```bash
# Hatch #1 — direct joint commands
ros2 control switch_controllers \
  --deactivate arm_left_jt_controller --activate arm_left_direct_cmd --strict
ros2 topic pub /arm_left_joint_cmd_direct \
  cynlr_arm_interfaces/msg/JointCommand \
  "{mode: 0, joint_position: [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0]}" --once
# Verify: ros2 topic echo /arm_left_arm_state --once  (joint_positions should shift)

# Switch back
ros2 control switch_controllers \
  --deactivate arm_left_direct_cmd --activate arm_left_jt_controller --strict

# Hatch #2 — NRT passthrough (arm's own planner)
ros2 control switch_controllers \
  --deactivate arm_left_jt_controller --activate arm_left_nrt --strict
ros2 action send_goal /arm_left_move_j cynlr_arm_interfaces/action/MoveJ \
  "{target_positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], max_joint_vel: 0.5, max_joint_acc: 1.0}"

# Switch back
ros2 control switch_controllers \
  --deactivate arm_left_nrt --activate arm_left_jt_controller --strict

# Hatch #3 — Cartesian command
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

## Key paths

| Item | Path |
|---|---|
| cynlr_arm_core install | `~/cynlr_software/cynlr_install/` |
| flexiv_description | `Cpp_App_Test/flexiv_description/` (humble branch, built with colcon) |
| Architecture doc | `thoughts/shared/docs/architecture.md` |
| Implementation plan | `thoughts/shared/plans/2026-04-13-robot-arm-service.md` |
| Claude conversation memory | `.claude/` (included in repo) |
