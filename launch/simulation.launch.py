from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'akrobat'

    ld.add_action(DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='wether to launch gazebo without gui (headless)'
    ))

    ld.add_action(DeclareLaunchArgument(
        'start_paused',
        default_value='false',
        description='wether to launch gazebo with paused physics'
    ))

    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='wether to launch rviz alongside for visualization'
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r -v4 empty.sdf']}.items()
    ))

    ld.add_action(IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'include', 'rviz.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('rviz'))
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
                Command(['xacro ', PathJoinSubstitution([FindPackageShare(package_name), 'urdf', 'akrobat.xacro']), ' simulation:=True']),
                value_type=str
            ),
            'use_sim_time': ParameterValue(True, value_type=bool)
        }]
    ))

    ld.add_action(Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', package_name,
                   '-z', '.2'],
        output='screen'
    ))

    ld.add_action(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager-timeout', ' 30'],
        output='screen'
    ))

    ld.add_action(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', ' 30'],
        output='screen'
    ))

    return ld