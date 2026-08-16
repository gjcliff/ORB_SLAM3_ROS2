from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_name",
                default_value="",
                description="Name of the database output file.",
            ),
            Node(
                package="orb_slam3_ros2",
                executable="visualize_node",
                output="screen",
                # prefix="xterm -e gdb --args",
                parameters=[
                    {
                        "output_name": LaunchConfiguration("output_name"),
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("orb_slam3_ros2"),
                            "config",
                            "visualization.rviz",
                        ]
                    ),
                ],
            ),
        ],
    )
