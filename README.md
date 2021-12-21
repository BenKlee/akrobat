# Akrobat

Control and visualization of a six-legged walking robot
based on ROS.

## How to make the robo walk in a simulator with a joystick

#### Prerequisites

- Working installation of ros-melodic (follow the installation instructions on [the official ROS website](http://wiki.ros.org/kinetic/Installation/Ubuntu) )

- Install the ROS Kinetic joystick 

```sudo apt-get install ros-kinetic-joy```

- Add your user to the dialout group (you need this because the Akrobat is controlled via the USB interface)

```sudo add <youruser> dialout``` 
 

#### Steps to get the simulation running

1. Get the repo into your catkin_ws

- ```cd ~/catkin_ws/src/```

- ```git clone -b <yourbranchname> github.com/informatik-mannheim/akrobat```

2. catkin_make your workspace

- ```cd ~/catkin_ws/```

- ```catkin_make```

3. roslaunch the project with your desired running options

There are two launch configurations

- ```master.launch```\
and
- ```gazebo.launch```

 ```master.launch``` is designed for launching the robot when using the actual physical robot
 ```gazebo.launch``` is designed to launch the robot in the gazebo simulation

Both launch configurations can be given arguments
- ```master.launch```
    - ```rviz```
        - launches the rviz visualization tool along with the robot
        - ```default: false```
    - ```robot_connected```
        - wether the robot is currently connected to the computer 
        - ```default: true```
        - turn off if just testing in rviz

- ```gazebo.launch```
    - ```world```
        - the world to launch the robot into in gazebo
        - ```default: "default"```
        - when changing from default, make sure there is a ```.world``` file in the ```/worlds``` folder corrensponding to the value you provided
    - ```gui```
        - wether to show the gazebo gui
        - ```default: true```
        - can be turned off to speed up simulation
    - ```rviz```
        - wether to launch rviz
        - ```default: false```
        - can be used to visualize the robot when gui is turned off


Troubleshooting:

If you get an error like "[...] in the folder [akrobat] couldn't be found a file [AkrobatMaster.launch] [...]" execute this line:

```echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc```
