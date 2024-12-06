# Author: Lucas Butler

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="llm_tools",
                executable="llm_tools",
                name="llm_tools",
                output="screen",
                parameters=[{
                    "include_services": [
                        "/*"
                    ]
                }]
            ),
            Node(
                package="llm_model",
                executable="chatgpt",
                name="chatgpt",
                output="screen",
                parameters=[{
                    "passiv_topics": [
                        "/cmd_vel:geometry_msgs/Twist",
                    ]
                }]
            ),
        ]
    )
