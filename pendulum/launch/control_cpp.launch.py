import launch
import launch_ros
from launch import LaunchDescription  
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  
from launch.substitutions import LaunchConfiguration  
from launch_ros.actions import Node  
from ament_index_python import get_package_share_directory

def generate_launch_description():  
    return LaunchDescription([  
        # Declare the controller argument  
        DeclareLaunchArgument('controller', default_value='LQR', description='Controller type'),  

        # Include the spawn_gazebo.launch file  
        IncludeLaunchDescription(  
            get_package_share_directory('pendulum') + '/launch/spawn_gazebo.launch.py'  
        ),  

        # Launch the control node  
        Node(  
            package='pendulum',  
            executable='control',  # ROS 2 中的可执行文件名称  
            name='control',  
            output='screen',  
            parameters=[{'controller': LaunchConfiguration('controller')}]  
        ),  
    ])
    
    