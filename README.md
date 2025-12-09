# aubo_ros2_driver Test Script

## Start up
```bash
source install/setup.bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py use_real_hardware:=true # for mock hardware, replace true to false
```

## Record topics
```bash
ros2 bag record -a
```