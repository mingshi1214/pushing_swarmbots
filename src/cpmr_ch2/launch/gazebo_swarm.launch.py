import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('cpmr_apb'), f'blockrobot.urdf.xacro')

    # Spawn 5 robots in Gazebo
    nodelist = []
    poselist = [[1.5, 1.37], [2.5, 1.37], [0.87, 2], [3.13, 2], [1.5, 2.63], [2.5, 2.63]]#[[1.5, 1.35], [2.5, 1.35], [0.85, 2], [3.15, 2], [1.5, 2.65], [2.5, 2.65]]
    for i in range(0, 6):
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
                            "-x", str(poselist[i][0]), 
                            '-y', str(poselist[i][1]), 
                            '-Y', '0']
            )
        )
        # nodelist.append(DeclareLaunchArgument(f'goal_x_{robot_name}', default_value = str(poselist[i][0]+3.0), description = 'goal (x)'))
        nodelist.append(DeclareLaunchArgument(f'goal_x_{robot_name}', default_value = '[1.0, -1.0, 1.0]', description = 'goal (x)'))
        # nodelist.append(DeclareLaunchArgument(f'goal_y_{robot_name}', default_value = str(poselist[i][1]+3.0), description = 'goal (y)'))
        nodelist.append(DeclareLaunchArgument(f'goal_y_{robot_name}', default_value = '[1.0, -1.0, 1.0]', description = 'goal (y)'))
        # nodelist.append(DeclareLaunchArgument(f'goal_t_{robot_name}', default_value = '[0.0, 0.0, 0.0]', description = 'goal (t)'))
        # nodelist.append(DeclareLaunchArgument(f'goal_t_{robot_name}', default_value = '[0.78, 0.78, 0.78]', description = 'goal (t)'))
        # nodelist.append(DeclareLaunchArgument(f'goal_t_{robot_name}', default_value = '[1.65, 1.65, 1.65]', description = 'goal (t)'))
        nodelist.append(DeclareLaunchArgument(f'goal_t_{robot_name}', default_value = '[2.43, 0.78, 1.65]', description = 'goal (t)'))
        # nodelist.append(DeclareLaunchArgument(f'goal_t_{robot_name}', default_value = '[3.14, 3.14, 3.14]', description = 'goal (t)'))
        # breaks on this:
        # nodelist.append(DeclareLaunchArgument(f'goal_t_{robot_name}', default_value = '[0.78, 0.78, 0.78]', description = 'goal (t)'))
        # nodelist.append(DeclareLaunchArgument(f'goal_t_{robot_name}', default_value = '[1.65, 1.65]', description = 'goal (t)'))
        nodelist.append(DeclareLaunchArgument(f'max_vel_{robot_name}', default_value = '0.5', description = 'max (v)'))
        nodelist.append(DeclareLaunchArgument(f'vel_gain_{robot_name}', default_value = '0.2', description = 'controller gain'))

        nodelist.append(DeclareLaunchArgument('object_x', default_value='[1.0, 3.0, 3.0, 1.0]', description='object vertices (x)'))
        nodelist.append(DeclareLaunchArgument('object_y', default_value='[1.5, 1.5, 2.5, 2.5]', description='object vertices (y)'))
        nodelist.append(
            Node(
                namespace=robot_name,
                package='cpmr_ch2',
                executable='swarm',
                name='swarm',
                output="screen",
                remappings=[('/odom', '/'+robot_name+"/odom"),
                            ('/cmd_vel', "/"+robot_name+"/cmd_vel"),
                            ('/diagnosis', "/"+robot_name+"/diagnosis"),
                            ('/complete_waypoint', "/"+robot_name+"/complete_waypoint")],
                parameters = [
                {'goal_x' : LaunchConfiguration(f'goal_x_{robot_name}')},
                {'goal_y' : LaunchConfiguration(f'goal_y_{robot_name}')},
                {'goal_t' : LaunchConfiguration(f'goal_t_{robot_name}')},
                {'max_vel' : LaunchConfiguration(f'max_vel_{robot_name}')},
                {'vel_gain' : LaunchConfiguration(f'vel_gain_{robot_name}')},
                {'object_x' : LaunchConfiguration('object_x')},
                {'object_y' : LaunchConfiguration('object_y')}
            ]
            )
        )

    nodelist.append(Node(
             package='cpmr_ch2',
             executable='swarm_broadcast',
             name='swarm_broadcast',
             output='screen'))
        
    ld = LaunchDescription([
            DeclareLaunchArgument(
                name='world',
                default_value="src/cpmr_ch2/worlds/box",
                description='Full path to the world model file to load'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
                launch_arguments={'world': LaunchConfiguration('world')}.items()),
            *nodelist])
    return ld