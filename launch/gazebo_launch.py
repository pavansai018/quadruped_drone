from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import Command, LaunchConfiguration
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_name = 'drone_v1.urdf'
    urdf = os.path.join(get_package_share_directory('quadruped_drone'), 'urdf', urdf_name)
    control_yaaml_file = os.path.join(get_package_share_directory(
        'quadruped_drone'
    ), 'config', 'ros2_control.yaml')
    robot_desc = ParameterValue(
        Command(['xacro ', urdf, ' ', 'ros2_control_yaml:=', control_yaaml_file]),
        value_type=str
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) if true',
    )
    gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r empty.sdf'
        }.items()

    )
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'drone_v1',
            '-allow-renaming', 'true',
            '-z', '0.5',
            # '-R', '3.14',
           
        ]

    )

    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ]
    )
    params = {
        'use_sim_time': use_sim_time,
        'robot_description': robot_desc,
    }
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params],
        arguments=[],
    )

    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    spawner_rotor_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'rotor_velocity_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )
    wait_for_control_manager = Node(
        package='quadruped_drone',
        executable='wait_for_controller_manager',
        output='screen',
    )
    spawn_after_controller_manager = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_control_manager,
            on_exit=[
                spawner_jsb,
                spawner_rotor_velocity_controller,
            ],
        ),
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)
    ld.add_action(wait_for_control_manager)
    ld.add_action(spawn_after_controller_manager)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld