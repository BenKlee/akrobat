# TODO

This file compiles all things that need work or ideas for further development.

## software

### gaits
- implement wave and ripple gait
- implement turning
- implement roll pitch yaw control

### autonomous navigation
- was implemented in ROS1 previously using move_base which is now Nav2
    - old implementation can be found in the [noetic-dev branch](https://github.com/informatik-mannheim/akrobat/tree/noetic-dev)
    - [Nav2 docs](https://docs.nav2.org/)
    - [Nav2 github](https://github.com/ros-navigation/navigation2)

### mapping
- was implemented in ROS1 previously using [rtab_map](https://wiki.ros.org/rtabmap_ros/noetic_and_newer)
    - old implementation can be found in the [noetic-dev branch](https://github.com/informatik-mannheim/akrobat/tree/noetic-dev)

### ros2_control

- dynamixel hardware interface only writes, cannot read
    - this leads to unwanted behaviour on startup where the robot tries to move all motors to position 0 and crashing into itself
    - [github issue](https://github.com/dynamixel-community/dynamixel_hardware/issues/90#issuecomment-2243505547) for this problem, with promised fix in [this pull request](https://github.com/dynamixel-community/dynamixel_hardware/pull/89)

### gazebo

- add missing sensors
    - depth camera (picoflexx)
    - stereo camera

### urdf

- properly set inertia and center of mass
    - currently inertia is calculated with the incorrect assumption that all links are cuboids
    - these are not very accurate but it does not seem to affect the simulation too much
    - in rviz see RobotModel -> Mass Properties for visualization
- integrate models of the new feet


## hardware

### front of robot

- replace front raspberry pi (stereopi)
    - front raspberry pi ethernet connection to other pi was unstable
    - suddenly stopped working completely

### sensors

- add transform from akrobat link to depth camera link (RoyaleInRos_link)
    - use `ros2 run tf2_tools view_frames`

- implement stereo camera