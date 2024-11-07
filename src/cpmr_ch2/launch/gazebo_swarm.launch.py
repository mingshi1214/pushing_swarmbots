import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('cpmr_apb'), f'blockrobot.urdf.xacro')
    # with open(urdf, 'r') as infp:
    #     robot_desc= infp.read()

    # Spawn five robots in Gazebo
    nodelist = []
    for i in range(0, 3):
        robot_name = f"block_robot_{i}"
        robot_desc = xacro.process_file(urdf, mappings={'name' : robot_name}).toxml()
        nodelist.append(Node(
                namespace = robot_name,
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': False, 
                             'robot_description': robot_desc, 
                             'frame_prefix': robot_name + "/"}],
                arguments=[urdf])
            )
        nodelist.append(
            Node(
                namespace = robot_name,
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='urdf_spawner',
                output='screen',
                arguments=["-topic", "/" + robot_name + "/robot_description",  
                           "-entity", robot_name, 
                           "-x", str(i * 2.0), 
                           '-y', '0', 
                           '-Y', '0']
            )
        )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
        ),
        *nodelist  # Unpack and add all spawn actions
    ])