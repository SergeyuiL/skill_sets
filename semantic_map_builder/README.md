# Service Definition

In ```skillsets_msg```

```shell
std_msgs/Header     header
string              cmd #build/save/switch/reloc, build cmd do not use others
string              mapdir #absolute path with no end slash
string              filename #map file name with no suffix (.pbstream and .pgm)
bool                accurate_initial_pose #is the initial pose accurate

geometry_msgs/Pose  initial_pose  #only switch cmd use this
---

bool success
string message
```

# CLI calling service

```shell
ros2 service call /build_semantic_map robot_interfaces/srv/SetSlam "header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
cmd: 'build'
mapdir: 'home/nvidia/maps/map_save'
filename: 'map_save'
accurate_initial_pose: false
initial_pose:
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}"
```

# Using example

```shell
ros2 run build_semantic_map build_semantic_map

ros2 service call /build_semantic_map skillsets_msg/srv/SetSlam "{cmd: 'build', mapdir: '/home/sg/workspace/top-down-map/data/0720', filename: 'map_save'}"
```