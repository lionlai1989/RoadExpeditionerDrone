# RoadExpeditionerDrone

**RoadExpeditionerDrone** is a **fully autonomous** UAV capable of SE(3) flight control, mapping,
localization, exploration, and motion planning in unknown GNSS-denied environments.

https://github.com/user-attachments/assets/8fca52ad-3435-4a47-8245-756b2bbf219e

---

## Algorithm Overview
The project covers the **"full stack"** of an autonomous drone, including:

- **Simultaneous Localization and Mapping (RTAB-Map)**: Provides globally consistent mapping and
  loop closure using RGB-D data to correct for odometry drift.

- **Visual Inertial Odometry (OpenVINS)**: Estimates local continuous odometry at high frequency
  using camera and IMU data. (work in progress)

- **Map Exploration (Frontier Detection)**: Identifies boundaries between known free space and
  unknown areas to guide autonomous exploration.

- **Path Planning (Nav2)**: Computes optimal, collision-free global paths navigating through the
  generated map.

- **Trajectory Planning (Minimum Snap)**: Generates smooth, continuous polynomial trajectories.

- **Flight Controller**: A low-level nonlinear tracking controller by T. Lee et al. that publishes
  motor speed commands to follow desired trajectories.

*(Note: It's just my personal opinion that this project covers the "full stack" of an autonomous
drone. I'm sure there are plenty of other things missing, so please let me know if I missed
anything. Thanks!)*

---

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

- **`red_bringup`**: The main entry point. Orchestrates the ROS 2 nodes and launches the Gazebo
  simulation environment.

- **`red_geometric_controller`**: Runs the low-level SE(3) geometric controller, publishing motor
  speed commands at 100 Hz to follow planned trajectories.

- **`red_gz_plugin`**: Custom Gazebo plugins bridging simulated quadrotors with ROS 2. It publishes
  sensor data (IMU at 250 Hz, RGB-D at 30 Hz) and subscribes to motor speeds.

- **`red_navigator`**: High-level navigation state machine. Combines frontier exploration, Nav2 path
  planning, and trajectory generation to output desired tracking poses at 5 Hz.

- **`red_perception`**: Handles RGB-D sensor streams and runs RTAB-Map for 3D mapping and loop
  closure.

- **`red_vio_estimator`**: Wraps OpenVINS to compute high-frequency visual-inertial odometry for
  local continuous state estimation. (work in progress)

---

## How to run

- Grab the simulation world
  [here](https://drive.google.com/file/d/14_7dYjFfjNmqW4hYJAOBKtPSC8jPMvhD/view?usp=sharing), unzip
  it, and move the `worlds/` folder into `src/red_bringup/`.

- Initialize git submodule OpenVINS at `src/open_vins` before building
    ```bash
    git submodule update --init --recursive
    ```

- Build and exec into the container

- Build the workspace and launch
    ```bash
    source /opt/ros/humble/setup.bash \
    && colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    && source install/setup.bash \
    && ros2 launch red_bringup simulation.launch.py world_path:=src/red_bringup/worlds/single_layer_maze.world world_name:=single_layer_maze_world
    ```

---

## References
Materials used during development and recommended reading.

### ROS 2 Humble and Gazebo Harmonic
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/getstarted/)

### RTAB-Map
- RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online
  Operation

### Nav2
- The Marathon 2: A Navigation System

### OpenVINS
- OpenVINS: A Research Platform for Visual-Inertial Estimation

### PX4
- [Hands-On Aerial Robotics Using PX4 and ROS 2](https://github.com/Dronecode/roscon-25-workshop)

## How to cite
Use GitHub's “Cite this repository” button on the repository page to get BibTeX, APA, or EndNote
entries generated from `CITATION.cff`.

For quick copy, an example one‑line BibTeX is:
```bibtex
@software{RoadExpeditionerDrone, author={Lai, Chih-An Lion}, title={The RoadExpeditionerDrone Project}, year={2026}, version={1.0.0}, url={https://github.com/lionlai1989/RoadExpeditionerDrone}}
```

## License and third‑party assets

- **Project source code license**: This repository is licensed under the GNU General Public
   License v3.0 (GPL-3.0). See `LICENSE`.

- **PX4 model assets**: Files in `src/red_gz_plugin/models/` (for example `x500`,
   `x500_base`, `x500_depth`, and `OakD-Lite`) are sourced from
   [PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models) and remain under their original
   BSD-3-Clause licenses. The original license files are preserved in each model directory.

- **Celebrity Dataset**: Downloaded from
  [Kaggle](https://www.kaggle.com/datasets/jessicali9530/celeba-dataset) and [MMLAB,
  CUHK](http://mmlab.ie.cuhk.edu.hk/projects/CelebA.html). Original authors: Ziwei Liu, Ping Luo,
  Xiaogang Wang, and Xiaoou Tang.

- **Animal Image Dataset (90 Different Animals)**: Downloaded from
  [Kaggle](https://www.kaggle.com/datasets/iamsouravbanerjee/animal-image-dataset-90-different-animals/).

- **Van Gogh Starry Night**: Downloaded from
  [Pixabay](https://pixabay.com/illustrations/starry-night-vincent-van-gough-1093721/). Artist:
  Vincent van Gogh, Uploader: Perlinator.
