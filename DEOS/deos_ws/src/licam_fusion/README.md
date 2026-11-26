# **licam_fusion**

A real-time ROS 2 package for 3D LiDAR-Camera object-level fusion and 3D bounding box estimation using C++20 and Eigen.

Projects 3D LiDAR point clouds onto 2D camera image coordinates using a single precomputed $3 \times 4$ projection matrix $P = K[R \mid t]$, matches points within 2D bounding boxes with occlusion-aware foreground filtering, rejects background clutter via $O(k)$ depth histogram clustering and exact median depth estimation (`std::nth_element`), and reconstructs 3D metric positions, dimensions, and pose covariances.

Supports **zero-copy intra-process composable node** deployment (`ComposableNodeContainer`) and real-time latency/synchronization telemetry via `diagnostic_updater`.

### **Architecture**

* **`include/licam_fusion/fusion_core.hpp`**: ROS-independent, zero-runtime-allocation C++20 fusion engine (`FusionCore`), frame-tagged strong types (`Point3<Frame>`, `Transform<To, From>`), `Expected<T, E>` error handling, and fixed-capacity `RingBuffer<T, N>`.
* **`include/licam_fusion/fusion_node.hpp` & `src/fusion_node.cpp`**: Thin ROS 2 component wrapper managing TF2, `ApproximateTime` synchronization with upper interval bounds, atomic immutable calibration snapshots, `PointCloud2` layout validation, and `unique_ptr` publishing.

### **IO**

* **input**
  * `/camera/color/camera_info` (`sensor_msgs/CameraInfo`): Camera intrinsic calibration parameters ($P$, $K$, and distortion $D$)
  * `/perception/stereo_detections` (`vision_msgs/Detection2DArray`): 2D bounding boxes
  * `/points_downsampled` (`sensor_msgs/PointCloud2`): 3D point cloud (`FLOAT32` $x, y, z$)
  * `tf` (`tf2_msgs/TFMessage`): Transform from `lidar_frame_id` to camera optical frame (and optional `fixed_frame_id` / `output_frame_id`)

* **output**
  * `/perception/detections_3d` (`vision_msgs/Detection3DArray`): 3D bounding boxes with metric positions ($X, Y, Z$), estimated dimensions, and $6 \times 6$ pose covariance (published on every synchronized frame, including empty arrays for downstream heartbeat monitoring)
  * `/perception/detections_3d_markers` (`visualization_msgs/MarkerArray`): RViz 3D bounding boxes and depth labels (prefixed with `DELETEALL` to prevent stale markers)
  * `/diagnostics` (`diagnostic_msgs/DiagnosticArray`): Real-time mean/p99 latency ($\mu\text{s}$), synchronization delta ($|t_{\text{cam}} - t_{\text{lidar}}|$ in ms), calibration status, and frame counters

### **Config**

Configured via `config/fusion_params.yaml`:

| NAME | TYPE | DEFAULT VALUE | DESCRIPTION |
| :--- | :--- | :--- | :--- |
| `camera_info_topic` | string | `"/camera/color/camera_info"` | Camera intrinsic calibration topic |
| `bbox_topic` | string | `"/perception/stereo_detections"` | 2D bounding box detections topic |
| `lidar_topic` | string | `"/points_downsampled"` | Target 3D point cloud topic |
| `output_3d_bbox_topic` | string | `"/perception/detections_3d"` | Output 3D bounding box array topic |
| `marker_topic` | string | `"/perception/detections_3d_markers"` | RViz visualization markers topic |
| `lidar_frame_id` | string | `"cloud"` | TF frame ID of LiDAR |
| `output_frame_id` | string | `""` | Output TF frame ID (empty = camera optical frame, e.g. `"base_link"`) |
| `fixed_frame_id` | string | `""` | Fixed frame (e.g. `"odom"`) for ego-motion-compensated TF lookup |
| `sync_queue_size` | int | `15` | ApproximateTime message queue size |
| `max_sync_interval_sec` | double | `0.05` | Maximum allowed timestamp gap $\|t_{\text{cam}} - t_{\text{lidar}}\|$ (seconds) |
| `detector_width` | int | `0` | 2D detector inference width (0 = same as `CameraInfo`) |
| `detector_height` | int | `0` | 2D detector inference height (0 = same as `CameraInfo`) |
| `use_rectified_image` | bool | `true` | Use projection matrix $P$ (`false` uses $K$ and plumb-bob distortion $D$) |
| `one_shot_camera_info` | bool | `false` | Unsubscribe from `CameraInfo` after first successful calibration |
| `min_points_per_bbox` | int | `5` | Minimum points required to validate a 3D detection |
| `cluster_tolerance` | double | `1.2` | Distance threshold (meters) for depth clustering |
| `min_depth` | double | `0.2` | Minimum depth (meters) — points closer are discarded |
| `max_depth` | double | `100.0` | Maximum depth (meters) — points farther are discarded |
| `box_shrink` | double | `0.9` | 2D box shrink factor $[0.1, 1.0]$ to suppress boundary background leakage |
| `bin_size` | double | `0.1` | Depth histogram bin resolution (meters) |
| `occlusion_handling` | bool | `true` | Exclude foreground occluder clusters from overlapping background boxes |
| `filter_ground` | bool | `false` | Filter ground points exceeding `max_camera_y` in camera optical frame |
| `max_camera_y` | double | `1.8` | Camera height threshold (meters, $+Y$ down) when `filter_ground` is enabled |
| `class_prior_weight` | double | `0.0` | Blending weight $[0.0, 1.0]$ between measured cluster extent and class size priors |
| `publish_markers` | bool | `true` | Enable/disable RViz marker generation |

### **Algorithmic Notes & Known Physical Characteristics**

* **3D Center & Median Depth:** Object $X, Y$ centers and $X, Y$ dimensions are computed from the actual 3D points belonging to the selected foreground cluster rather than back-projecting the 2D box center. For even cluster sizes, `nth_element(n / 2)` selects the upper median index.
* **Depth Extent (`size.z`):** Because a single-view LiDAR only observes the front-facing surface of an opaque object, raw depth spread underestimates full vehicle length; set `class_prior_weight > 0.0` to blend measured spread with class-specific dimensional priors (`car`, `truck`, `bus`, `pedestrian`, `cyclist`).

### **Setup, Test & Run**

```bash
colcon build --packages-select licam_fusion --symlink-install
colcon test --packages-select licam_fusion && colcon test-result --verbose
source install/setup.bash
ros2 launch licam_fusion fusion.launch.py
```

#### **Command-line overrides**

```bash
ros2 launch licam_fusion fusion.launch.py lidar_topic:=/velodyne_points lidar_frame_id:=velodyne output_frame_id:=base_link
```

#### **Composable node (zero-copy intra-process)**

`fusion.launch.py` automatically deploys `licam_fusion::FusionNode` inside a `ComposableNodeContainer` with `use_intra_process_comms:=true`. To load into an existing container:

```bash
ros2 component load /licam_fusion_container licam_fusion licam_fusion::FusionNode -e use_intra_process_comms:=true
```
