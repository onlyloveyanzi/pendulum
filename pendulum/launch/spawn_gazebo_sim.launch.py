import launch
import launch_ros
from ament_index_python import get_package_share_directory
import launch_ros.parameter_descriptions
from launch.substitutions import LaunchConfiguration  
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    model_name = 'pendulum_sim'
    
    declare_x = DeclareLaunchArgument(  
        'x', default_value='0.0', description='X position of the robot'  
    )  
    declare_y = DeclareLaunchArgument(  
        'y', default_value='0.0', description='Y position of the robot'  
    )  
    declare_z = DeclareLaunchArgument(  
        'z', default_value='0.0', description='Z position of the robot'  
    )  
    # declare_robot_name = DeclareLaunchArgument(  
    #     'robot_name', default_value='pendulum', description='Name of the robot'  
    # )  
    default_xacro_path = get_package_share_directory('pendulum') + '/urdf/cartpole.urdf.xacro'
    default_gazeobo_path = get_package_share_directory('pendulum') + '/world/empty_world.world'  

    #声明一个urdf目录的参数，方便修改
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_xacro_path),description='xacro path'
    )
    
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    ) 
    
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description}],
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
                '-topic', '/robot_description' , 
                '-entity', model_name ,                
            ] ,
        output='screen'
    )
    
        # 加载并激活 fishbot_joint_state_broadcaster 控制器
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'cartpole_joint_state_broadcaster'],
        output='screen'
    )
    
    load_x_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'x_controller'],
        output='screen'
    )

    return launch.LaunchDescription([
        declare_x,  
        declare_y,  
        declare_z,  
        # declare_robot_name,  
        action_declare_arg_model_path,
        action_robot_state_publisher,
        action_launch_gazeobo,
        action_spawn_entity,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[load_joint_state_controller],)
            ),
        # 事件动作，load_fishbot_diff_drive_controller
        launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_x_controller],)
            ),
    ])