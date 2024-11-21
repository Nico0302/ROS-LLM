# Author: Lucas Butler

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            # Node(
            #     package="llm_input",
            #     executable="llm_audio_input",
            #     name="llm_audio_input",
            #     output="screen",
            # ),
            Node(
                package="llm_model",
                executable="chatgpt",
                name="chatgpt",
                output="screen",
            ),
            # Node(
            #     package="llm_output",
            #     executable="llm_audio_output",
            #     name="llm_audio_output",
            #     output="screen",
            # ),
            Node(
                package="llm_robot",
                executable="turtle_robot",
                name="turtle_robot",
                output="screen",
            ),
            Node(
                package="turtlesim",
                executable="turtlesim_node",
                name="turtlesim_node",
                output="screen",
            ),
        ]
    )
