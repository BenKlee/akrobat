# Akrobat

Control and visualization of a six-legged walking robot
based on ROS.

## How to make the robo walk in a simulator with a joystick

#### Prerequisites

- Working installation of ros-noetic (follow the installation instructions on [the official ROS website](http://wiki.ros.org/noetic/Installation/Ubuntu) )

- Add your user to the dialout group (you need this because the Akrobat is controlled via the USB interface)

`sudo add <youruser> dialout` 
 

#### Steps to get the simulation running

1. Get the repo into your catkin_ws

- `cd ~/catkin_ws/src/`

- `git clone -b <yourbranchname> github.com/informatik-mannheim/akrobat`

2. Install dependencies

- `rosdep install akrobat`

3. catkin_make your workspace

- this needs to be done every time you change something in the code (does not include launch files or things like worlds, urdf, stl or yaml-configs)

- `cd ~/catkin_ws/`

- `catkin_make`

4. roslaunch the project with your desired running options

#### Launch Configurations

There are two launch configurations

- `master.launch`\
and
- `gazebo.launch`

 `master.launch` is designed for launching the robot when using the actual physical robot\
 `gazebo.launch` is designed to launch the robot in the gazebo simulation

Both launch configurations can be given arguments. Run either\
`roslaunch akrobat master.launch --ros-args`\
or\
`roslaunch akrobat gazebo.launch --ros-args`\
to see their arguments' descriptions and default values


For testing purposes without a controller (joystick) available, a joystick emulator has been implemented. To use it run `rosrun akrobat joystick_emulator.py`


Troubleshooting:

If you get an error like "[...] in the folder [akrobat] couldn't be found a file [master.launch] [...]" execute this line:

`echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`