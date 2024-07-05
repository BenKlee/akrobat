from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription,  RegisterEventHandler
from launch.actions import  IncludeLaunchDescription
from launch.substitutions import  PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'akrobat'


    ld.add_action(IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'include', 'rviz.launch.py'])
        )
    ))

    ld.add_action(Node(
        package=package_name,
        executable='akrobat',
        output='screen'
    ))

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', PathJoinSubstitution([FindPackageShare(package_name), 'urdf', 'akrobat.xacro']), ' test:=True']),
                value_type=str
            ),
            'use_sim_time': True
        }]
    ))

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[PathJoinSubstitution([FindPackageShare('akrobat'), 'config', 'ros2_control.yaml'])],
        remappings=[('/controller_manager/robot_description', '/robot_description')],
        # arguments=['--ros-args', '--log-level', 'debug'],
        output='screen'
    )
    ld.add_action(controller_manager)

    # delay controller spawn until after controller manager is started
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessStart(
            on_start=Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller'],
                output='screen'
            ),
            target_action=controller_manager
        )
    ))

    # delay controller spawn until after controller manager is started
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessStart(
            on_start=Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            ),
            target_action=controller_manager
        )
    ))

    return ld