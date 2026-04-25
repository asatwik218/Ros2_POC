from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    # ── Declare arguments ──────────────────────────────────────────────────────

    declared_arguments = [
        DeclareLaunchArgument(
            "sn_left",
            default_value="",
            description="Serial number of the left arm (empty = use vendor sim).",
        ),
        DeclareLaunchArgument(
            "sn_center",
            default_value="",
            description="Serial number of the center arm (empty = use vendor sim).",
        ),
        DeclareLaunchArgument(
            "sn_right",
            default_value="",
            description="Serial number of the right arm (empty = use vendor sim).",
        ),
        DeclareLaunchArgument(
            "vendor",
            default_value="sim",
            description="Hardware vendor: 'sim' for SimArm, 'flexiv' for real hardware.",
            choices=["sim", "flexiv"],
        ),
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
    ]

    sn_left    = LaunchConfiguration("sn_left")
    sn_center  = LaunchConfiguration("sn_center")
    sn_right   = LaunchConfiguration("sn_right")
    vendor     = LaunchConfiguration("vendor")
    use_moveit = LaunchConfiguration("use_moveit")
    use_rviz   = LaunchConfiguration("use_rviz")

    # ── Robot description (xacro → URDF) ──────────────────────────────────────

    cynlr_urdf_xacro = PathJoinSubstitution(
        [FindPackageShare("cynlr_arm_description"), "urdf", "cynlr_arm_system.urdf.xacro"]
    )

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                cynlr_urdf_xacro,
                " sn_left:=",    sn_left,
                " sn_center:=",  sn_center,
                " sn_right:=",   sn_right,
                " vendor:=",     vendor,
            ]
        ),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    # ── Semantic robot description (SRDF) ─────────────────────────────────────

    cynlr_srdf_xacro = PathJoinSubstitution(
        [FindPackageShare("cynlr_moveit_config"), "srdf", "cynlr_three_arm.srdf.xacro"]
    )

    robot_description_semantic_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                cynlr_srdf_xacro,
            ]
        ),
        value_type=str,
    )

    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # ── Controller YAML ────────────────────────────────────────────────────────

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("cynlr_bringup"), "config", "cynlr_controllers.yaml"]
    )

    # ── Core nodes ────────────────────────────────────────────────────────────

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ParameterFile(robot_controllers, allow_substs=True)],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, robot_description_semantic],
    )

    # ── RViz ─────────────────────────────────────────────────────────────────

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(use_rviz),
    )

    # ── MoveIt move_group ─────────────────────────────────────────────────────

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

    # ── Controller spawners ───────────────────────────────────────────────────
    # Spawners exit after the controller is activated. We chain them with
    # OnProcessExit to enforce startup ordering without polling or sleep.

    def spawner(name, *, inactive=False):
        args = [name, "--controller-manager", "/controller_manager"]
        if inactive:
            args += ["--inactive"]
        return Node(package="controller_manager", executable="spawner", arguments=args)

    # Always-active ────────────────────────────────────────────────────────────
    jsb_spawner          = spawner("joint_state_broadcaster")
    left_state_spawner   = spawner("arm_left_state_broadcaster")
    center_state_spawner = spawner("arm_center_state_broadcaster")
    right_state_spawner  = spawner("arm_right_state_broadcaster")

    # Default motion path (MoveIt / JTC) ──────────────────────────────────────
    left_jt_spawner   = spawner("arm_left_jt_controller")
    center_jt_spawner = spawner("arm_center_jt_controller")
    right_jt_spawner  = spawner("arm_right_jt_controller")

    # Escape hatches — loaded but inactive; switch via ros2 control switch_controllers
    left_direct_spawner   = spawner("arm_left_direct_cmd",   inactive=True)
    center_direct_spawner = spawner("arm_center_direct_cmd", inactive=True)
    right_direct_spawner  = spawner("arm_right_direct_cmd",  inactive=True)

    left_nrt_spawner   = spawner("arm_left_nrt",   inactive=True)
    center_nrt_spawner = spawner("arm_center_nrt", inactive=True)
    right_nrt_spawner  = spawner("arm_right_nrt",  inactive=True)

    left_cart_spawner   = spawner("arm_left_cartesian",   inactive=True)
    center_cart_spawner = spawner("arm_center_cartesian", inactive=True)
    right_cart_spawner  = spawner("arm_right_cartesian",  inactive=True)

    # ── Startup chain (OnProcessExit) ─────────────────────────────────────────
    #
    # jsb → left_state → center_state → right_state
    #     → left_jt → center_jt → right_jt
    #     → escape hatches (inactive) → rviz, moveit
    #
    # Each spawner exits once its controller is ACTIVE, so the next step only
    # fires after the previous controller is fully running.

    delay_left_state_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[left_state_spawner],
        )
    )

    delay_center_state_after_left_state = RegisterEventHandler(
        OnProcessExit(
            target_action=left_state_spawner,
            on_exit=[center_state_spawner],
        )
    )

    delay_right_state_after_center_state = RegisterEventHandler(
        OnProcessExit(
            target_action=center_state_spawner,
            on_exit=[right_state_spawner],
        )
    )

    delay_left_jt_after_right_state = RegisterEventHandler(
        OnProcessExit(
            target_action=right_state_spawner,
            on_exit=[left_jt_spawner],
        )
    )

    delay_center_jt_after_left_jt = RegisterEventHandler(
        OnProcessExit(
            target_action=left_jt_spawner,
            on_exit=[center_jt_spawner],
        )
    )

    delay_right_jt_after_center_jt = RegisterEventHandler(
        OnProcessExit(
            target_action=center_jt_spawner,
            on_exit=[right_jt_spawner],
        )
    )

    # Load all escape-hatch controllers (inactive) after the last JTC is up
    delay_hatches_after_right_jt = RegisterEventHandler(
        OnProcessExit(
            target_action=right_jt_spawner,
            on_exit=[
                left_direct_spawner,   center_direct_spawner,   right_direct_spawner,
                left_nrt_spawner,      center_nrt_spawner,      right_nrt_spawner,
                left_cart_spawner,     center_cart_spawner,     right_cart_spawner,
            ],
        )
    )

    # RViz and MoveIt after the right cartesian spawner exits (all controllers loaded)
    delay_viz_after_hatches = RegisterEventHandler(
        OnProcessExit(
            target_action=right_cart_spawner,
            on_exit=[rviz_node, moveit_node],
        )
    )

    # ── Assemble LaunchDescription ────────────────────────────────────────────

    nodes = [
        ros2_control_node,
        robot_state_publisher_node,
        jsb_spawner,
        delay_left_state_after_jsb,
        delay_center_state_after_left_state,
        delay_right_state_after_center_state,
        delay_left_jt_after_right_state,
        delay_center_jt_after_left_jt,
        delay_right_jt_after_center_jt,
        delay_hatches_after_right_jt,
        delay_viz_after_hatches,
    ]

    return LaunchDescription(declared_arguments + nodes)
