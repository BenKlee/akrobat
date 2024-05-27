from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()


    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='wether to launch rviz alongside for visualization'
    ))

    ld.add_action(DeclareLaunchArgument(
        'gazebo',
        default_value='False',
        description='wether to launch gazebo simulator'
    ))

    ld.add_action(DeclareLaunchArgument(
        'using_real_robot',
        default_value='True',
        description='wether to start sending commands to the robots\' motors'
    ))

    # ld.add_action(IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         launch_file_path=PathJoinSubstitution([FindPackageShare('akrobat'), 'launch', 'include', 'dynamixel.launch.py'])
    #     ),
    #     condition=IfCondition(LaunchConfiguration('using_real_robot'))
    # ))

    ld.add_action(IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('akrobat'), 'launch', 'include', 'rviz.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('rviz'))
    ))

    ld.add_action(IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([FindPackageShare('akrobat'), 'launch', 'include', 'gazebo.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('gazebo'))
    ))


    ld.add_action(Node(
        package='akrobat',
        executable='akrobat',
        output='screen'
    ))

    return ld