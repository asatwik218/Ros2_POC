import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("cynlr_arm_service")
    config_name = LaunchConfiguration("config").perform(context)
    config_file = os.path.join(pkg_share, "config", config_name)

    arm_node = LifecycleNode(
        package="cynlr_arm_service",
        executable="arm_service_node",
        name="arm_service_node",
        namespace=LaunchConfiguration("namespace").perform(context),
        parameters=[config_file],
        output="screen",
    )

    return [arm_node]


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="arm",
        description="ROS2 namespace for the arm service node",
    )

    config_arg = DeclareLaunchArgument(
        "config",
        default_value="sim.yaml",
        description="Config file name (relative to package config/ dir)",
    )

    return LaunchDescription([
        namespace_arg,
        config_arg,
        OpaqueFunction(function=launch_setup),
    ])
