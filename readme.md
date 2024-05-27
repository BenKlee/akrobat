### Install dependencies
```
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
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