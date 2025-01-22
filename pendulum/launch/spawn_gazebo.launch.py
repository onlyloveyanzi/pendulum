import launch
import launch_ros

from launch import LaunchDescription  
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration  
from launch_ros.actions import Node  
import os  
from ament_index_python import get_package_share_directory

def generate_launch_description():  
    return LaunchDescription([  
        # Gazebo Arguments  
        DeclareLaunchArgument('paused', default_value='false', description='Start Gazebo paused'),  
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),  
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo with GUI'),  
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo in headless mode'),  
        DeclareLaunchArgument('debug', default_value='false', description='Enable debug mode'),  

        # Spawning arguments  
        DeclareLaunchArgument('x', default_value='0.0', description='X position of the robot'),  
        DeclareLaunchArgument('y', default_value='0.0', description='Y position of the robot'),  
        DeclareLaunchArgument('z', default_value='0.0', description='Z position of the robot'),  
        DeclareLaunchArgument('robot_name', default_value='pendulum', description='Name of the robot'),  
        DeclareLaunchArgument('robot_file', default_value=os.path.join(get_package_share_directory('pendulum'), 'urdf', 'cartpole.urdf'), description='Path to the robot URDF file'),  

        # Load robot description parameter  
        Node(  
            package='xacro',  
            executable='xacro',  
            name='xacro',  
            output='screen',  
            parameters=[{'robot_description': LaunchConfiguration('robot_file')}],  
            arguments=[LaunchConfiguration('robot_file')]  
        ),  

        # Launch Gazebo with own world configuration  
        IncludeLaunchDescription(  
            get_package_share_directory('pendulum') + '/launch/empty_world.launch.py',  
            launch_arguments={  
                'world_name': 'worlds/empty.world',  
                'debug': LaunchConfiguration('debug'),  
                'gui': LaunchConfiguration('gui'),  
                'paused': LaunchConfiguration('paused'),  
                'use_sim_time': LaunchConfiguration('use_sim_time'),  
                'headless': LaunchConfiguration('headless'),  
            }.items()  
        ),  

        # Spawn the ROBOT  
        Node(  
            package='gazebo_ros',  
            executable='spawn_model',  
            name='urdf_spawner',  
            output='screen',  
            arguments=[  
                '-urdf',  
                '-x', LaunchConfiguration('x'),  
                '-y', LaunchConfiguration('y'),  
                '-z', LaunchConfiguration('z'),  
                '-model', LaunchConfiguration('robot_name'),  
                '-param', '/robot_description'  
            ]  
        ),  

        # Load joint controller configurations  
        Node(  
            package='controller_manager',  
            executable='spawner',  
            name='controller_spawner',  
            output='screen',  
            arguments=['joint_state_controller', 'x_controller'],  
            namespace='/pendulum'  
        ),  

        # Load the robot state publisher  
        Node(  
            package='robot_state_publisher',  
            executable='robot_state_publisher',  
            name='robot_state_publisher',  
            output='screen',  
            remappings=[('/joint_states', '/pendulum/joint_states')]  
        ),  
    ])