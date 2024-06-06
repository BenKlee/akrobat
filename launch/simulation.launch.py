from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'run_headless',
        default_value='False',
        description='wether to launch gazebo without gui (headless)'
    ))

    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='wether to launch rviz alongside for visualization'
    ))

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', PathJoinSubstitution([FindPackageShare('akrobat'), 'urdf', 'akrobat.xacro']), ' simulation:=True']),
                value_type=str
            ),
            'use_sim_time': ParameterValue(True, value_type=bool)
        }
        ]
    ))

    ld.add_action(IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments=[('gui:=False')],
        condition=IfCondition(LaunchConfiguration('run_headless'))
    ))

    ld.add_action(IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'pause': 'true'}.items(),
        condition=UnlessCondition(LaunchConfiguration('run_headless'))
    ))

    ld.add_action(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'akrobat',
                   '-z', '2'],
        output='screen'
    ))

    ld.add_action(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_position_controller'],
        output='screen'
    ))

    ld.add_action(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    ))

    ld.add_action(Node(
        package='akrobat',
        executable='akrobat',
        output='screen'
    ))

    ld.add_action(IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('akrobat'), 'launch', 'include', 'rviz.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('rviz'))
    ))

    return ld