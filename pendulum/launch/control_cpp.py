from launch import LaunchDescription  
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  
from launch.substitutions import LaunchConfiguration, FindPackageShare  
from launch_ros.actions import Node  

def generate_launch_description():  
    return LaunchDescription([  
        # Declare the controller argument  
        DeclareLaunchArgument('controller', default_value='LQR', description='Controller type'),  

        # Include the spawn_gazebo.launch file  
        IncludeLaunchDescription(  
            FindPackageShare('pendulum_control') + '/launch/spawn_gazebo.launch.py'  
        ),  

        # Launch the control node  
        Node(  
            package='pendulum_control',  
            executable='control',  # ROS 2 中的可执行文件名称  
            name='control',  
            output='screen',  
            parameters=[{'controller': LaunchConfiguration('controller')}]  
        ),  
    ])
    
    
import os