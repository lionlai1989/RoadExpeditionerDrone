# RoadExpeditionerDrone - ROS 2 + Gazebo + OpenVINS + Nav2 + RTAB-Map

**RoadExpeditionerDrone** is a fully containerized **ROS 2** + **Gazebo** project that builds a
fully autonomous drone perform mapping, localization, exploration and planning. 

## Architecture

The repository includes six in-house ROS packages under `src/`:
```
src/
  red_bringup/
  red_geometric_controller/
  red_gz_plugin/
  red_navigator/
  red_perception/
  red_vio_estimator/
```

- `src/red_bringup`: Launches the full simulation environment and orchestrates ROS processes.
- `src/red_geometric_controller`: Runs a low-level geometric controller node and publishes motor
  speed commands at 100 Hz.
- `src/red_gz_plugin`: Gazebo plugin that bridges motor speed commands. IMU updates at 250 Hz and
  camera updates at 30 Hz.
- `src/red_navigator`:
  - `navigator.cpp`: Maintains the navigation state machine and publishes the desired pose commands
    at 5 Hz.
  - `frontier_explorer.cpp`: Library for frontier detection in the map.
- `src/red_perception`: Bridges RGB and depth sensors and launches RTAB-Map.
- `src/red_vio_estimator`: Currently bridges Gazebo ground-truth odometry. The current goal of this
  project is to replace that source with OpenVINS estimation.

## How to run

### OpenVINS
This project has a git submodule OpenVINS at `src/open_vins`. After cloning this repository,
initialize submodules before building:
```bash
git submodule update --init --recursive
```

### Activate Environment
Open Cursor, run "Dev Containers: Rebuild and Reopen in Container".

### Fresh container setup (from scratch)
Follow these steps after the container is rebuilt:

1. Clean build
```
rm -rf build install log ~/.ros/rtabmap.db
```

2. Build the workspace and launch
```bash
source /opt/ros/humble/setup.bash \
&& colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
&& source install/setup.bash \
&& ros2 launch red_bringup simulation.launch.py world_path:=src/red_bringup/worlds/single_layer_maze.world world_name:=single_layer_maze_world
```

## Useful debug commands
```bash
# Visualize nodes, topics, services
rqt_graph
```

list all ros topics:
```
$ ros2 topic list
```

list all ros nodes:
```
ros2 node list
```

list add ros transforms:
```
ros2 run tf2_tools view_frames
```

get frequency of a topic:
```
ros2 topic hz /drone_1/odometry
```

list all gz topics:
```
gz topic -l
```

monitor a gz topic:
```
gz topic -i -t /drone_1/command/motor_speed
```

shows all models in the gz sim
```
gz model --list
```

gz topic list
```
gz topic -i -t /drone_1/command/motor_speed
gz model -m drone_1 -l
```

---

## References
Materials used during development and recommended reading.

### ROS 2 Humble and Gazebo Harmonic
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/getstarted/)

### RTAM-Map

### Nav2

### OpenVINS

### Drone-related
- [Hands-On Aerial Robotics Using PX4 and ROS 2](https://github.com/Dronecode/roscon-25-workshop)
