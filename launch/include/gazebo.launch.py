from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'run_headless',
        default_value='False',
        description='wether to launch gazebo without gui (headless)'
    ))


    ld.add_action(IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments=[('gz_args', '-s empty.sdf')],
        condition=IfCondition(LaunchConfiguration('run_headless'))
    ))
    ld.add_action(IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments=[('gz_args', 'empty.sdf')],
        condition=UnlessCondition(LaunchConfiguration('run_headless'))
    ))

    ld.add_action(Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', Command(['xacro ', PathJoinSubstitution([FindPackageShare('akrobat'), 'urdf', 'akrobat.xacro'])])],
        output='screen'
    ))

    return ld