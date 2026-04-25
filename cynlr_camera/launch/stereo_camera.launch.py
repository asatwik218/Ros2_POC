from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "camera_index",
            default_value="0",
            description="Camera index for multi-camera setups.",
        ),
        DeclareLaunchArgument(
            "config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("cynlr_camera"), "config", "camera_config.yaml"]
            ),
            description="Path to camera parameter YAML.",
        ),
    ]

    camera_node = Node(
        package="cynlr_camera",
        executable="stereo_camera_node",
        name="stereo_camera_node",
        namespace=["camera_", LaunchConfiguration("camera_index")],
        parameters=[LaunchConfiguration("config")],
        output="both",
    )

    return LaunchDescription(declared_arguments + [camera_node])
