cmake_minimum_required(VERSION 3.15)
project(cynlr_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmState.msg"
  "msg/JointCommand.msg"
  "msg/CartesianCommand.msg"
  "msg/FaultEvent.msg"
  "msg/OperationalStatus.msg"
  "srv/Connect.srv"
  "srv/SetMode.srv"
  "srv/SetTool.srv"
  "srv/Trigger.srv"
  "srv/SetDigitalOutputs.srv"
  "srv/GetDigitalInputs.srv"
  "srv/SwitchController.srv"
  "action/MoveL.action"
  "action/MoveJ.action"
  "action/MovePTP.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
