### Install dependencies
```
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

#### Install dynamixel_control (ros2_control hardware interface)

```
cd ~/ros2_ws/src
git clone https://github.com/youtalk/dynamixel_control.git -b humble dynamixel
vcs import dynamixel < dynamixel/dynamixel_control.repos
rosdep install --from-paths dynamixel --ignore-src -r -y
cd ~/ros2_ws/
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```


### Build & source
```
colcon build && source install/setup.bash
```


### Launch


#### List launch arguments
```
ros2 launch akrobat main.launch.py -s
```

#### Launch
```
ros2 launch akrobat main.launch.py
```