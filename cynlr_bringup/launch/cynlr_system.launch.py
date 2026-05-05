import os
import subprocess
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

def load_system_config(config_path: str) -> dict:
    with open(config_path) as f:
        return yaml.safe_load(f)


def generate_urdf(config: dict) -> str:
    """Build a top-level xacro from config, run xacro, return URDF string."""
    macro_path = os.path.join(
        get_package_share_directory("cynlr_arm_description"),
        "urdf", "cynlr_rizon7_macro.urdf.xacro"
    )

    arms_xml = ""
    for arm in config["arms"]:
        xyz  = " ".join(str(v) for v in arm["mount"]["xyz"])
        rpy  = " ".join(str(v) for v in arm["mount"]["rpy"])
        tool = arm.get("tool", {})
        com      = " ".join(str(v) for v in tool.get("com",      [0.0, 0.0, 0.0]))
        inertia  = " ".join(str(v) for v in tool.get("inertia",  [0.0] * 6))
        tcp_pose = " ".join(str(v) for v in tool.get("tcp_pose", [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
        arms_xml += f"""
  <xacro:cynlr_rizon7
    prefix="{arm['name']}_"
    serial_number="{arm['serial_number']}"
    vendor="{arm['vendor']}"
    parent="world"
    origin_xyz="{xyz}"
    origin_rpy="{rpy}"
    tool_mass_kg="{tool.get('mass_kg', 0.0)}"
    tool_com="{com}"
    tool_inertia="{inertia}"
    tool_tcp_pose="{tcp_pose}"
  />"""

    xacro_str = f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="cynlr_arm_system">
  <xacro:include filename="{macro_path}"/>
  <link name="world"/>
{arms_xml}
</robot>"""

    with tempfile.NamedTemporaryFile(suffix=".urdf.xacro", mode="w", delete=False) as tf:
        tf.write(xacro_str)
        tf_path = tf.name

    try:
        result = subprocess.run(
            ["xacro", tf_path],
            capture_output=True, text=True, check=True
        )
        return result.stdout
    finally:
        os.unlink(tf_path)


def generate_controller_yaml(config: dict, params_file_path: str) -> str:
    """Generate the controller_manager YAML from config and return as string.

    In ROS2 Jazzy, controller nodes do not inherit the process --params-file.
    The controller_manager reads '{controller_name}.params_file' from its own
    parameter section and passes that file to each controller node's NodeOptions.
    We therefore embed a params_file reference pointing back at ourselves so the
    CM can route the right parameters to each dynamically-loaded controller.
    """
    update_rate = config["system"]["update_rate"]

    arm_prefixes = [f"{a['name']}_" for a in config["arms"]]

    cm: dict = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": update_rate,
                "arm_prefixes": arm_prefixes,
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
            }
        }
    }

    for arm in config["arms"]:
        n = arm["name"]
        ros_params = cm["controller_manager"]["ros__parameters"]

        # params_file tells the CM where to load per-controller parameters from
        ros_params[f"{n}_jt_controller"] = {
            "type": "joint_trajectory_controller/JointTrajectoryController",
            "params_file": params_file_path,
        }
        ros_params[f"{n}_direct_cmd"] = {
            "type": "cynlr_direct_command_controller/CynlrDirectCommandController",
            "params_file": params_file_path,
        }
        ros_params[f"{n}_cartesian"] = {
            "type": "cynlr_cartesian_controller/CynlrCartesianController",
            "params_file": params_file_path,
        }

        def joints(name=n):
            return [f"{name}_joint{i}" for i in range(1, 8)]

        cm[f"{n}_jt_controller"] = {
            "ros__parameters": {
                "joints": joints(),
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
                "state_publish_rate": 50.0,
                "action_monitor_rate": 20.0,
                "allow_partial_joints_goal": False,
                "constraints": {
                    "stopped_velocity_tolerance": 0.01,
                    "goal_time": 5.0,
                },
            }
        }
        cm[f"{n}_direct_cmd"] = {
            "ros__parameters": {"prefix": f"{n}_", "joints": joints()}
        }
        cm[f"{n}_cartesian"] = {
            "ros__parameters": {"prefix": f"{n}_"}
        }

    return yaml.dump(cm, default_flow_style=False)


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_moveit",
            default_value="false",
            description="Launch MoveIt move_group node.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz for visualization.",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=os.path.join(
                get_package_share_directory("cynlr_bringup"),
                "config", "cynlr_system_config.yaml"
            ),
            description="Path to cynlr_system_config.yaml.",
        ),
    ]

    use_moveit = LaunchConfiguration("use_moveit")
    use_rviz   = LaunchConfiguration("use_rviz")

    def launch_setup(context, *args, **kwargs):
        config_path = LaunchConfiguration("config_file").perform(context)
        config = load_system_config(config_path)

        # ── URDF ────────────────────────────────────────────────────────────
        urdf_str = generate_urdf(config)
        robot_description = {"robot_description": urdf_str}

        # ── Controller YAML → tempfile ───────────────────────────────────────
        # Create the tempfile first so we know its path, then embed the path
        # as 'params_file' inside each controller's CM entry (Jazzy mechanism).
        ctrl_tmp = tempfile.NamedTemporaryFile(
            suffix=".yaml", mode="w", delete=False, prefix="cynlr_ctrl_"
        )
        ctrl_tmp_path = ctrl_tmp.name
        ctrl_yaml = generate_controller_yaml(config, ctrl_tmp_path)
        ctrl_tmp.write(ctrl_yaml)
        ctrl_tmp.flush()
        ctrl_tmp.close()

        # ── Semantic description (SRDF) ──────────────────────────────────────
        from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
        from launch_ros.substitutions import FindPackageShare
        from launch_ros.parameter_descriptions import ParameterValue

        cynlr_srdf_xacro = PathJoinSubstitution(
            [FindPackageShare("cynlr_moveit_config"), "srdf", "cynlr_three_arm.srdf.xacro"]
        )
        robot_description_semantic_content = ParameterValue(
            Command([FindExecutable(name="xacro"), " ", cynlr_srdf_xacro]),
            value_type=str,
        )
        robot_description_semantic = {
            "robot_description_semantic": robot_description_semantic_content
        }

        # ── cynlr_main: ControllerManager + one CynlrArmNode per arm ────────
        # All run in the same process so they share the CynlrArmRegistry singleton.
        # arm_prefixes are passed via the 'arm_prefixes' parameter in the controller YAML.
        cynlr_main_node = Node(
            package="cynlr_arm_node",
            executable="cynlr_main",
            parameters=[robot_description, ParameterFile(ctrl_tmp_path, allow_substs=True)],
            output="both",
        )

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, robot_description_semantic],
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            condition=IfCondition(use_rviz),
        )

        moveit_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="both",
            parameters=[
                robot_description,
                robot_description_semantic,
                PathJoinSubstitution(
                    [FindPackageShare("cynlr_moveit_config"), "config", "kinematics.yaml"]
                ),
                PathJoinSubstitution(
                    [FindPackageShare("cynlr_moveit_config"), "config", "moveit_controllers.yaml"]
                ),
                PathJoinSubstitution(
                    [FindPackageShare("cynlr_moveit_config"), "config", "ompl_planning.yaml"]
                ),
                PathJoinSubstitution(
                    [FindPackageShare("cynlr_moveit_config"), "config", "sensors_3d.yaml"]
                ),
            ],
            condition=IfCondition(use_moveit),
        )

        # ── Spawner helper ────────────────────────────────────────────────────
        def spawner(name, *, inactive=False):
            args = [name, "--controller-manager", "/controller_manager"]
            if inactive:
                args += ["--inactive"]
            return Node(
                package="controller_manager",
                executable="spawner",
                arguments=args,
            )

        # ── Spawner chain: jsb → jt[0..N-1] → escape hatches → viz ──────────
        arms = config["arms"]

        jsb_spawner = spawner("joint_state_broadcaster")
        jt_spawners = [spawner(f"{a['name']}_jt_controller") for a in arms]
        hatch_spawners = []
        for a in arms:
            n = a["name"]
            hatch_spawners += [
                spawner(f"{n}_direct_cmd", inactive=True),
                spawner(f"{n}_cartesian",  inactive=True),
            ]

        event_handlers = []

        # jsb → jt[0]
        event_handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[jt_spawners[0]],
        )))

        # jt[i] → jt[i+1]
        for i in range(len(jt_spawners) - 1):
            event_handlers.append(RegisterEventHandler(OnProcessExit(
                target_action=jt_spawners[i],
                on_exit=[jt_spawners[i + 1]],
            )))

        # last jt → all escape-hatch spawners at once
        event_handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=jt_spawners[-1],
            on_exit=hatch_spawners,
        )))

        # last hatch → rviz + moveit
        event_handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=hatch_spawners[-1],
            on_exit=[rviz_node, moveit_node],
        )))

        # Cleanup tempfile on shutdown
        event_handlers.append(RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda ctx: os.unlink(ctrl_tmp_path)
                         if os.path.exists(ctrl_tmp_path) else None)]
        )))

        return [
            cynlr_main_node,
            robot_state_publisher_node,
            jsb_spawner,
            *event_handlers,
        ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
