# Container-based Development

## Source of Odometry

In the `src/red_vio_estimator/src/odom_adapter.cpp` module, odometry data is bridged from the Gazebo
simulation environment or computed by an estimator. The system architecture defines three primary
modes of operation for odometry controlled by `vio_source`: `groundtruth`, `synthetic`, and
`openvins`.

### 1. Groundtruth mode (`vio_source:=groundtruth`)

Groundtruth mode uses the simulator pose directly (bridged from Gazebo) as the odometry source. This
groundtruth odometry is expressed in the Gazebo `world` frame and serves as a perfect, noise-free
representation of the drone's position and orientation.

It is effectively "perfect odometry":

- No sensor noise
- No bias drift
- No scale error
- No tracking loss

Current state: The entire system, including the geometric controller and trajectory planner,
functions perfectly in this mode.

This mode is excellent for early bring-up to:

- Verify TF wiring and topic remapping
- Validate controller signs/axes and planner integration
- Confirm that failures are not caused by state-estimation uncertainty

Limitation: While it proves the fundamental logic of our control algorithms, it creates a false
sense of security. Real-world sensors are never perfect, and success in this mode does not guarantee
the system is robust or practically viable. A system that only works with unrealistic clean odometry
may still fail in realistic VIO conditions.

### 2. OpenVINS mode (`vio_source:=openvins`)

OpenVINS mode is designed to simulate a real-world deployment where the drone relies entirely on
Visual-Inertial Odometry (VIO). It starts with a brief initialization using groundtruth data but
rapidly transitions to compute its odometry using the OpenVINS algorithm (visual-inertial estimation
from camera + IMU). Unlike groundtruth mode, this is an estimator output with real failure modes:

- Short-term noise and jitter
- Slowly accumulating drift
- Sensitivity to texture, motion profile, and timing
- Startup/transient behavior during initialization

Key Characteristic: A critical design requirement for OpenVINS mode is that it **must completely
break away from the Gazebo `world` frame**. Once initialized, OpenVINS establishes its own internal
global reference frame based on its initial visual-inertial state, independent of the simulation's
absolute coordinate system. This accurately reflects how a drone operates in the wild without
external positioning systems (like GNSS, GPS, or motion capture).

Current State: The system can fail when operating in OpenVINS mode due to the complexities
introduced.

### 3. Synthetic mode (`vio_source:=synthetic`)

To bridge the gap between perfection (Groundtruth Mode) and total failure (OpenVINS Mode), we have
an intermediate testing ground. Directly debugging the system while it relies on OpenVINS is
exceptionally difficult because:

1. OpenVINS introduces complex, interconnected errors (noise, biases, drift).
2. It's hard to pinpoint whether a failure is caused by a fundamental flaw in the control logic or
   simply the controller's inability to handle a specific type or magnitude of VIO error.

`groundtruth` is too clean and `openvins` can fail for many coupled reasons at once. `synthetic`
mode mimics the characteristics of OpenVINS odometry systematically. We achieve this by taking the
perfect Gazebo groundtruth odometry and intentionally degrading it before feeding it to the rest of
the system.

#### Requirements for Synthetic Odometry

To accurately represent real-world VIO, the synthetic odometry module relies on three main
corruptions:
- **Noises**: High-frequency, random fluctuations mimicking sensor noise (e.g., Gaussian noise on
  position and velocity).
- **Biases**: Constant or slowly varying offsets in the measurements.
- **Random Walks**: The accumulation of error over time, causing the estimated position to gradually
  diverge from the true position, even when the drone is stationary.

This allows us to:

- Isolate downstream module robustness (controller, navigator, planner) in isolation without
  estimator complexity.
- Reproduce failures deterministically by using fixed parameter settings.
- Sweep error magnitude gradually to find exact breakpoints.
- Build a staged validation pipeline before enabling full OpenVINS.

#### Design and Workflow

To fulfill these requirements, the following design is implemented in
`src/red_vio_estimator/src/odom_adapter.cpp` and should be used as a workflow when operating
`vio_source:=synthetic`:

1. **Independent Global Frame (Anchor to local odometry frame)**
   - Similar to `openvins` mode, when initialized, `synthetic` odometry totally breaks away from the Gazebo `world` frame.
   - Use first received groundtruth pose as synthetic origin $(0, 0, 0)$ with identity rotation for the synthetic odometry frame.
   - Publish all later synthetic poses in local `odom` coordinates (relative to this local origin), not absolute Gazebo `world`.

2. **Generate synthetic odometry from local groundtruth**
   - Start from `groundtruth_local`.
   - Apply configurable corruption terms (ROS configurable standard deviations):
     - `noise_pos_stddev` (m): High-frequency noise added directly to position.
     - `noise_vel_stddev` (m/s): High-frequency noise added directly to linear velocity.
     - `random_walk_pos_stddev` (m/s): Standard deviation for position drift per step.
     - `random_walk_yaw_stddev` (rad/s): Standard deviation for yaw drift per step.
   - Conceptually:
     `synthetic = groundtruth_local + noise + bias + random_walk`

3. **Publish at the same rate as groundtruth callback**
   - Generate and publish synthetic odometry reactively inside the `groundtruth_odometry` callback.
   - This guarantees it publishes at the exact same rate as the simulation groundtruth, keeping timestamps aligned.

4. **Run a progressive robustness campaign**
   - Phase A: zero corruption (should match groundtruth behavior)
   - Phase B: add small noise only
   - Phase C: increase noise and add drift
   - Phase D: stress levels near expected OpenVINS error envelope

5. **Compare and diagnose**
   - Monitor `/groundtruth_odometry` vs `/odometry` divergence.
   - Track when mission metrics degrade (tracking error, oscillation, path quality, recovery failures).
   - Tune downstream modules based on measured sensitivity.

6. **Promote to OpenVINS**
   - Only switch to full `openvins` testing after synthetic stress tests are stable.
   - This reduces OpenVINS debugging to estimator-specific issues instead of mixed-system failures.

## Coordinate Frames

Following standard ROS conventions (REP-105), the operational TF chain is
`map -> odom -> base_link -> sensor frames`.

- **Gazebo `world`**: The absolute global frame used by the simulator for model poses and ground
  truth. It is not part of the ROS TF chain unless explicitly published.

- **OpenVINS `global`**: An estimator-internal reference frame, independent of Gazebo `world`.
  During initialization, it is gravity-aligned (global +Z opposite gravity), while yaw and position
  gauge freedoms are set by initialization constraints.

- **`odom`**: A local, continuous frame used for control and short-horizon tracking. In this
  project, `odom_adapter` publishes `odom -> base_link`, initializes `odom` to identity at startup,
  bootstraps from ground truth, and then (in OpenVINS mode) hands off to OpenVINS with a frozen
  alignment transform to preserve continuity.

- **`map`**: A globally consistent SLAM frame produced by RTAB-Map. RTAB-Map publishes `map ->
  odom`; this transform may jump during loop closure to correct long-term drift.

- **`base_link`**: The drone body frame (REP-103: x forward, y left, z up). Sensor frames (IMU,
  cameras) are static children of `base_link` directly or via intermediate frames (e.g.,
  `camera_link`).

## Useful Commands

- Visualize nodes, topics, services
```
rqt_graph
```

- List all ROS topics:
```
~/RoadExpeditionerDrone$ ros2 topic list
/clicked_point
/clock
/diagnostics
/drone_1/apriltag/detections
/drone_1/aruco/detections
/drone_1/aruco_opencv/detections
/drone_1/bond
/drone_1/cloud_ground
/drone_1/cloud_map
/drone_1/cloud_obstacles
/drone_1/command/motor_speed
/drone_1/depth/camera_info
/drone_1/depth/image
/drone_1/desired_odometry
/drone_1/global_costmap/costmap
/drone_1/global_costmap/costmap_raw
/drone_1/global_costmap/costmap_updates
/drone_1/global_costmap/footprint
/drone_1/global_costmap/global_costmap/transition_event
/drone_1/global_costmap/published_footprint
/drone_1/global_path
/drone_1/global_path_nodes
/drone_1/global_pose
/drone_1/goal
/drone_1/goal_node
/drone_1/goal_out
/drone_1/goal_reached
/drone_1/gps/fix
/drone_1/grid_prob_map
/drone_1/groundtruth_odometry
/drone_1/imu
/drone_1/info
/drone_1/initialpose
/drone_1/labels
/drone_1/landmark_detection
/drone_1/landmark_detections
/drone_1/landmarks
/drone_1/local_grid_empty
/drone_1/local_grid_ground
/drone_1/local_grid_obstacle
/drone_1/local_path
/drone_1/local_path_nodes
/drone_1/localization_pose
/drone_1/loop_depth
/drone_1/loop_depth/compressed
/drone_1/loop_depth/compressedDepth
/drone_1/loop_depth/theora
/drone_1/loop_depth_colored
/drone_1/loop_depth_colored/compressed
/drone_1/loop_depth_colored/compressedDepth
/drone_1/loop_depth_colored/theora
/drone_1/loop_extrinsic
/drone_1/loop_feats
/drone_1/loop_intrinsics
/drone_1/loop_pose
/drone_1/map
/drone_1/mapData
/drone_1/mapGraph
/drone_1/mapOdomCache
/drone_1/mapPath
/drone_1/map_updates
/drone_1/octomap_binary
/drone_1/octomap_empty_space
/drone_1/octomap_full
/drone_1/octomap_global_frontier_space
/drone_1/octomap_grid
/drone_1/octomap_ground
/drone_1/octomap_obstacles
/drone_1/octomap_occupied_space
/drone_1/odometry
/drone_1/odomimu
/drone_1/pathgt
/drone_1/pathimu
/drone_1/plan
/drone_1/planner_server/transition_event
/drone_1/points_aruco
/drone_1/points_msckf
/drone_1/points_sim
/drone_1/points_slam
/drone_1/posegt
/drone_1/poseimu
/drone_1/rgb/camera_info
/drone_1/rgb/image
/drone_1/rtabmap/republish_node_data
/drone_1/tag_detections
/drone_1/trackhist
/drone_1/trackhist/compressed
/drone_1/trackhist/compressedDepth
/drone_1/trackhist/theora
/drone_1/unsmoothed_plan
/drone_1/user_data_async
/drone_1/visual_path
/initialpose
/move_base_simple/goal
/parameter_events
/rosout
/tf
/tf_static
```

- List all ROS nodes:
```
~/RoadExpeditionerDrone$ ros2 node list
/drone_1/depth_bridge
/drone_1/flight_interface
/drone_1/geometric_controller_node
/drone_1/global_costmap/global_costmap
/drone_1/imu_bridge
/drone_1/lifecycle_manager_navigation
/drone_1/navigator_node
/drone_1/odom_adapter
/drone_1/odometry_groundtruth_bridge
/drone_1/openvins
/drone_1/planner_server
/drone_1/rgb_bridge
/drone_1/rtabmap
/drone_1/rviz2
/drone_1/static_tf_base_link_to_StereoOV7251
/drone_1/static_tf_base_link_to_camera_link
/drone_1/static_tf_base_link_to_imu_sensor
/drone_1/static_tf_base_link_to_imx214
/drone_1/transform_listener_impl_55e163ea3220
/drone_1/transform_listener_impl_56a3497839a0
/drone_1/transform_listener_impl_58208091ee00
/drone_1/transform_listener_impl_650ded189030
/ros_gz_bridge_clock
```

- List all ROS transforms:
```
# save to frames_*.gv
~/RoadExpeditionerDrone$ ros2 run tf2_tools view_frames
```

- Get frequency of a ROS topic:
```
~/RoadExpeditionerDrone$ ros2 topic hz /drone_1/odometry
average rate: 162.661
        min: 0.002s max: 0.174s std dev: 0.01471s window: 164
average rate: 196.888
        min: 0.001s max: 0.174s std dev: 0.00957s window: 396
average rate: 199.207
        min: 0.001s max: 0.174s std dev: 0.00841s window: 600
```

- List all Gazebo topics:
```
~/RoadExpeditionerDrone$ gz topic -l
/clock
/drone_1/command/motor_speed
/gazebo/resource_paths
/gui/camera/pose
/gui/currently_tracked
/gui/track
/model/drone_1/odometry
/model/drone_1/odometry_with_covariance
/model/drone_1/pose
/sensors/marker
/stats
/world/single_layer_maze_world/clock
/world/single_layer_maze_world/dynamic_pose/info
/world/single_layer_maze_world/model/drone_1/link/base_link/sensor/imu_sensor/imu
/world/single_layer_maze_world/model/drone_1/link/base_link/sensor/navsat_sensor/navsat
/world/single_layer_maze_world/model/drone_1/link/camera_link/sensor/IMX214/camera_info
/world/single_layer_maze_world/model/drone_1/link/camera_link/sensor/IMX214/image
/world/single_layer_maze_world/model/drone_1/link/camera_link/sensor/StereoOV7251/camera_info
/world/single_layer_maze_world/model/drone_1/link/camera_link/sensor/StereoOV7251/depth_image
/world/single_layer_maze_world/model/drone_1/link/camera_link/sensor/StereoOV7251/depth_image/points
/world/single_layer_maze_world/pose/info
/world/single_layer_maze_world/scene/deletion
/world/single_layer_maze_world/scene/info
/world/single_layer_maze_world/state
/world/single_layer_maze_world/stats
/world/single_layer_maze_world/light_config
/world/single_layer_maze_world/material_color
```

- Check a topic info
```
~/RoadExpeditionerDrone$ ros2 topic info /drone_1/groundtruth_odometry
Type: nav_msgs/msg/Odometry
Publisher count: 1
Subscription count: 1

~/RoadExpeditionerDrone$ ros2 topic info /drone_1/odomimu
Type: nav_msgs/msg/Odometry
Publisher count: 1
Subscription count: 1
```

- Echo a topic
```
~/RoadExpeditionerDrone$ ros2 topic echo /drone_1/groundtruth_odometry --once
header:
  stamp:
    sec: 26
    nanosec: 474000000
  frame_id: drone_1/odom
child_frame_id: base_link
pose:
  pose:
    position:
      x: 0.1717624200014549
      y: 0.14759456979226276
      z: 1.004849135096039
    orientation:
      x: 0.0004421677389328969
      y: 6.664824350618467e-05
      z: 0.9354640663051916
      w: 0.3534215340036009
  covariance:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
twist:
  twist:
    linear:
      x: 0.005936595370283374
      y: -0.011820595552447837
      z: -0.0023790858835462704
    angular:
      x: 0.001200939294909607
      y: -0.00015051510292427134
      z: 0.29768048411510095
  covariance:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
---
~/RoadExpeditionerDrone$ ros2 topic echo /drone_1/odomimu --once
header:
  stamp:
    sec: 48
    nanosec: 236000000
  frame_id: global
child_frame_id: imu
pose:
  pose:
    position:
      x: 524.0059590795693
      y: -541.8659050516916
      z: 2162.3691493474594
    orientation:
      x: 0.011868988924405377
      y: -0.12840298366496253
      z: -0.7841384711993249
      w: 0.6070409037890555
  covariance:
  - 125.86681079318883
  - 121.12481361786037
  - -0.10303018309036126
  - 0.03038978631846393
  - 0.05113505057067531
  - 0.2247365431072873
  - 121.12481361785991
  - 117.1587417458563
  - 0.022560989359674703
  - 0.029520597865216647
  - 0.04920014339809374
  - 0.21673994769454033
  - -0.10303018309035923
  - 0.022560989359670207
  - 0.028395545024802297
  - 2.9537003361377188e-05
  - -4.229477996344877e-05
  - -8.736810491777887e-05
  - 0.030389786318464077
  - 0.029520597865216595
  - 2.953700336137925e-05
  - 7.694322930646216e-06
  - 1.2258688431100008e-05
  - 5.448035152536218e-05
  - 0.051135050570675245
  - 0.04920014339809386
  - -4.229477996344797e-05
  - 1.225868843109973e-05
  - 2.1090461771056878e-05
  - 9.163272426984228e-05
  - 0.22473654310728747
  - 0.21673994769454036
  - -8.736810491776139e-05
  - 5.4480351525361866e-05
  - 9.163272426984202e-05
  - 0.00040211217278125007
twist:
  twist:
    linear:
      x: 86.37960064937997
      y: 134.4579131110566
      z: 215.30644600072247
    angular:
      x: 0.06602473296943537
      y: 0.05597980272247064
      z: -0.3010309613216407
  covariance:
  - 2.9755371683157197
  - -1.9263867887540203
  - 0.00794957761217231
  - 0.0
  - 0.0
  - 0.0
  - -1.9263867887540265
  - 1.253001513906738
  - -0.008114578778154679
  - 0.0
  - 0.0
  - 0.0
  - 0.007949577612173667
  - -0.00811457877815541
  - 0.001547766084588071
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 7.617600000004222e-07
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 7.617600000004222e-07
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 7.617600000004222e-07
---
```
