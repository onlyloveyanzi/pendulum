import launch  
from launch import LaunchDescription  
from launch_ros.actions import Node  

def generate_launch_description():  
    return LaunchDescription([  
        Node(  
            package='gazebo_ros',  
            executable='gzserver',  
            name='gazebo',  
            output='screen',  
            arguments=['-s', 'libgazebo_ros_init.so']  
        ),  
        Node(  
            package='gazebo_ros',  
            executable='gzclient',  
            name='gazebo_client',  
            output='screen'  
        )  
    ])