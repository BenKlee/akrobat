from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package='dynamixel_akrobat_python',
        executable='dynamixel_control',
        name='dynamixel_control',
        parameters=[
            ParameterFile('$(find akrobat)/config/dynamixel_config.yaml')
        ]
    ))

    return ld