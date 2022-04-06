### Project Structure
```
akrobat
│
├── CMakeLists.txt
│     - see documentation (http://wiki.ros.org/catkin/CMakeLists.txt)
├── config
│   ├── akrobat_control.yaml
│   │         -  config file for ros_gazebo_control (http://gazebosim.org/tutorials/?tut=ros_control)
│   └── dynamixel_params.yaml
│             - config file for dynamixel
├── include
│     - folder for all header files of C++ files in src folder
├── launch
│   ├── gazebo.launch
│   │         - launch file to launch akrobat in gazbeo
│   ├── master.launch
│   │         - launch file to launch akrobat normally
│   └── include      
│             - folder for launch files used within main launch files
├── msg
│     - folder for custom message files used in this project
├── nodes
│     - folder for all python nodes
├── package.xml
│     - defines dependencies
│     - see documentation (http://wiki.ros.org/catkin/package.xml)
├── rviz
│   └── akrobat.rviz
│         - rviz configuration
├── setup.py
├── src
│     - folder for all C++ files/nodes
├── stl
│     - folder for 3D models of the robot
├── urdf
│     - folder for urdf/xacro description of robot (http://wiki.ros.org/urdf)
└── worlds
      - folder for all world files used in gazebo
```
[generated using tree](http://mama.indstate.edu/users/ice/tree/)