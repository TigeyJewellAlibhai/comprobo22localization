Launch the Simulator

```
ros2 launch neato2_gazebo neato_gauntlet_world.py
```

Run Teleop
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Launch rviz
```
rviz2
```

Start particle filter
```
ros2 launch robot_localization test_pf.py map_yaml:=path-to-your-yaml-file
```
