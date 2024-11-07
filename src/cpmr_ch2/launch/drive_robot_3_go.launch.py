import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value = '0.0', description = 'goal (x)'),
        DeclareLaunchArgument('goal_y', default_value = '0.0', description = 'goal (y)'),
        Node(
            package = 'cpmr_ch2',
            executable = 'drive_to_goal_3',
            name = 'drive_to_goal_3',
            parameters = [
                {'goal_x' : LaunchConfiguration('goal_x')},
                {'goal_y' : LaunchConfiguration('goal_y')},
            ],
        ),
    ])

