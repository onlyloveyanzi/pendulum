import launch
import launch_ros
from ament_index_python import get_package_share_directory
import launch_ros.parameter_descriptions
from launch.substitutions import LaunchConfiguration  
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    declare_x = DeclareLaunchArgument(  
        'x', default_value='0.0', description='X position of the robot'  
    )  
    declare_y = DeclareLaunchArgument(  
        'y', default_value='0.0', description='Y position of the robot'  
    )  
    declare_z = DeclareLaunchArgument(  
        'z', default_value='0.0', description='Z position of the robot'  
    )  
    declare_robot_name = DeclareLaunchArgument(  
        'robot_name', default_value='pendulum', description='Name of the robot'  
    )  
    default_urdf_path = get_package_share_directory('pendulum') + '/urdf/cartpole.urdf'
    default_gazeobo_path = get_package_share_directory('pendulum') + '/world/empty_world.world'
    config_file_path = get_package_share_directory('pendulum_control')+'urdf'+ 'controller_config.yaml'  

    # 声明一个URDF目录的参数，方便修改  
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(  
        name='model', default_value=str(default_urdf_path), description='URDF path'  
    )  

    robot_description = launch_ros.parameter_descriptions.ParameterValue(  
        launch.substitutions.Command(['cat ',launch.substitutions.LaunchConfiguration('model')]),  # 直接使用URDF路径  
        value_type=str  
    )  
    
    
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        respawn=True,
        # arguments=[default_urdf_path],
        parameters=[{'robot_description':robot_description}],
        remappings=[('/joint_states', '/pendulum/joint_states')],
    )
    
    action_launch_gazeobo = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
        [get_package_share_directory('gazebo_ros'),'/launch/gazebo.launch.py']
    ),
    launch_arguments=[('world',default_gazeobo_path),('verbose','true')]
    )
    
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[  
                '-entity', 'pendulum' ,
                '-topic', 'robot_description' , 
                '-x', LaunchConfiguration('x'),  
                '-y', LaunchConfiguration('y'),  
                '-z', LaunchConfiguration('z'),   
                
            ] ,
        output='screen'
    )
    
    
    action_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_controller', 'x_controller'],
        namespace='/pendulum',
    )
           

    return launch.LaunchDescription([
        declare_x,  
        declare_y,  
        declare_z,  
        declare_robot_name,  
        action_declare_arg_model_path,
        action_robot_state_publisher,
        action_launch_gazeobo,
        action_spawn_entity,
        action_controller_spawner,
    ])