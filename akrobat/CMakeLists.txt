cmake_minimum_required(VERSION 3.0.2)
project(akrobat)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  joy
  rospy
  std_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  movement.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  #dynamixel_msgs#
  sensor_msgs     
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS dynamixel_workbench_toolbox dynamixel_workbench_controllers dynamixel_workbench_msgs dynamixel_sdk joy rospy std_msgs tf controller_manager hardware_interface message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  nodes/main.py
  nodes/translate_joypad.py
  nodes/joystick_emulator.py
  nodes/generate_world_file.py
  nodes/joint_states_to_gazebo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)