# Demo

```
colcon build --packages-select akrobat && . install/local_setup.bash
```

### simple.launch.py
```
ros2 launch akrobat simple.launch.py
```

### simulation.launch.py
```
ros2 launch akrobat simulation.launch.py
```

### change direction
```
ros2 topic pub --once /gait_direction geometry_msgs/msg/Point "{x: 0, y: 1, z: 0}"
```

### change speed
```
ros2 topic pub --once /gait_speed std_msgs/msg/Float64 "{data: 2}"
```

### change gait
```
ros2 topic pub --once /gait std_msgs/msg/String "{data: 't_pose'}"
```

### echo imu
```
ros2 topic echo /imu/data
```