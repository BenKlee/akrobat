from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart

from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()


    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='wether to launch rviz alongside for visualization'
    ))

    ld.add_action(IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('royale_in_ros2'), 'launch', 'camera_driver.launch.py'])
        )
    ))

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', PathJoinSubstitution([FindPackageShare('akrobat'), 'urdf', 'akrobat.xacro']), ' simulation:=False']),
                value_type=str
            )
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

    ld.add_action(IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('akrobat'), 'launch', 'include', 'rviz.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('rviz'))
    ))

    ld.add_action(Node(
        package='akrobat',
        executable='akrobat',
        output='screen'
    ))

    return ld