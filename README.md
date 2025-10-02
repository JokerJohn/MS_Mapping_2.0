# MS-Mapping Project Overview

> 中文文档请见 [README_ZH.md](README_ZH.md)。

## 1. Introduction

MS-Mapping is a ROS-based multi-session LiDAR SLAM framework designed for large-scale environments that require repeated mapping and localization. It fuses LiDAR odometry, GNSS observations, and loop closures to maintain a globally consistent pose graph, while providing tooling for map management, visualization, and data export.

### Project Motivation

- In the original **lio-sam_6axis** project, many users requested GNSS-integrated loop optimization; however, the underlying LIO could be unstable, often leading to mapping failures.
- **ms-mapping** brought robust multi-session capabilities, but remained research-oriented and lacked GNSS integration, making tasks such as initialization less convenient.
- This project is engineered as a practical, GNSS-centric mapping system. We integrate only the features that prove reliable in daily field work, regardless of how “cool” they sounded in the originating papers.

As a result, MS-Mapping has become my go-to system for real-world mapping: barring extreme scenarios, mapping failures are now exceedingly rare.

## 2. Key Capabilities

- **Multi-session Localization** – Seamlessly relocalize new sessions against a previously optimized map, with configurable baselines (Map-to-Frame or Frame-to-Frame).
- **Sensor Fusion** – Integrates LiDAR odometry, GNSS constraints (XYZ / full pose / XY-only), and optional IMU factors (Fast-LIO pipeline).
- **Robust Loop Closure** – Supports intra-session and cross-session loop detection with ICP, Scan Context, visual cues, and configurable thresholds.
- **Adaptive Initialization** – Accepts initial poses from YAML, RViz `initialpose` messages, or RTK GNSS; includes fallback strategies and yaw sweep search.
- **Map Management** – Provides tools to rebuild global maps, extract local submaps, and export trajectories, covariances, and point clouds.
- **Operational Telemetry** – Rich ROS logging, real-time RViz visualization (pose graph, GNSS constraints, loop edges, local/global maps).

## 3. System Architecture

```
ROS
 ├── ms_mapping (core pose graph node)
 │   ├── pose_slam(): real-time LiDAR SLAM loop
 │   ├── InitSystem(): multi-source initialization pipeline
 │   ├── OptimizeGraph(): ISAM2 incremental optimization
 │   └── VisualizationThread(): RViz publishing (loops, GNSS, map)
 │
 ├── cloud_process
 │   ├── ICP / GICP registration helpers
 │   ├── Local / global submap builders
 │   └── SC & feature extraction utilities
 │
 ├── data_saver
 │   ├── Pose / point cloud serialization
 │   ├── Factor graph persistence (g2o, TUM, ROS bag)
 │   └── Offline map reconstruction scripts
 │
 └── fastlio (optional backend)
     ├── IMU preprocessing
     ├── Deskewed point cloud generation
     └── High-rate lidar odometry
```

## 4. Module Breakdown

| Module | Description |
| --- | --- |
| `ms_mapping` | Core pose-graph SLAM node that subscribes to LiDAR odometry, GNSS, IMU, and manages keyframes, optimization, and visualization threads. |
| `cloud_process` | Handles point-cloud preprocessing, ICP/GICP alignment, loop detection helpers, and submap generation. |
| `data_saver` | Provides offline data export to TUM, g2o, ROS bag, and rebuilds archived maps for multi-session use. |
| `fastlio` (optional) | Fast-LIO front-end for IMU integration and deskewed clouds, feeding the pose graph. |
| `launch/` | Ready-to-run launch files (e.g., `ms_robosense.launch`, `ms_robosense_ps.launch`) with ROS bag playback and RViz setup. |
| `config/yaml/` | Parameter sets for different sensor suites (Robosense, Pandar, Mid360, etc.), including multi-session defaults. |
| `config/rviz/` | RViz layouts highlighting pose graph edges, GNSS constraints, point clouds, and diagnostic markers. |
| `scripts/` | Utility scripts for bag conversion, trajectory visualization, grid-map generation, and more. |

## 5. Initialization Module

The `InitSystem()` pipeline inside `ms_mapping` orchestrates multi-source initialization:

- Loads prior poses from YAML when `initial_pose_type=0` and validates parameters.
- Accepts live poses from RViz (`/initialpose`) while optionally republishing prior maps for user alignment.
- Handles GNSS-based alignment (`initial_pose_type=2`), including fallback yaw estimation, yaw sweep search, and local map extraction around the best candidate.
- Performs multi-attempt ICP (parallelized yaw hypotheses) to refine the initial transform before inserting the first node and factors into the pose graph.

This module guarantees a consistent starting state regardless of whether the session is brand-new or relocalizing against historic data.

## 6. Typical Workflow

1. **Prepare Data** – Record or obtain ROS bag files containing LiDAR, GNSS, and IMU topics.
2. **Configure** – Select or customize the YAML configuration (`pgo`, `common`, `lio` sections) to match sensors and desired baseline (F2F / M2F).
3. **Launch** – Use provided launch files (e.g., `ms_robosense.launch`) to start ms_mapping, auxiliary nodes, RViz, and optional bag playback.
4. **Monitor** – Observe real-time RViz overlays: keyframes, loop edges, GNSS constraints (green), and local/global maps.
5. **Optimize & Save** – The pose graph optimizes online; optional services allow map saving (`/save_map`), exporting data, or restarting sessions.
6. **Multi-session Run** – For subsequent sessions, load prior maps (`map_directory`) and relocalize using GNSS/RVIZ initial poses; loop closures and GNSS constraints keep consistency.

## 7. Configuration Highlights

- `pgo/baseline`: `0` for Frame-to-Frame or `1` for Map-to-Frame relocalization.
- `pgo/useMultiMode`: Enable multi-session logic and prior map loading.
- `pgo/historyKeyframeSearchTimeDiff`: Time gating for loop candidate filtering; adjust for replaying same-day bags.
- `common/useGPS` & `common/gnss_constraint_type`: Control GNSS fusion mode (XYZ / PriorPose / XY only).
- `common/initial_pose_type`: Choose YAML (0), RViz (1), or GNSS (2) initialization source.
- `lio/*`: Fast-LIO front-end tuning (if used).

## 8. Visualization

- **Loop Constraints** – Yellow/amber edges for intra-session loops and cyan for cross-session loops.
- **GNSS Factors** – Green markers/lines clearly distinguish GNSS constraints from loop closures.
- **Local Map** – Localized point cloud submaps published during initialization and loop verification.
- **Global Map** – Downsampled keyframe map for multi-session alignment, published on `/pgo_map`.

## 9. Logging & Diagnostics

- Console logs with color-coded events (e.g., green “ADD GPS Factor” messages) make it easy to trace GNSS integration.
- Timing statistics (`t1–t5`) reported every 10 frames help benchmark initialization, factor insertion, optimization, and more.
- Warnings throttle repetitive messages (e.g., invalid GNSS data, NaN loop poses) to keep logs actionable.
- The `/save_map` service provides progress feedback and persists artifacts under `save_directory/sequence/`.

## 10. Getting Started

```bash
# Clone & build
cd ~/catkin_ws/src
git clone <repository>
catkin build ms_mapping

# Run with sample configuration
roslaunch ms_mapping ms_robosense.launch

# (Optional) trigger map saving
rosservice call /save_map
```

## 11. Additional Resources

- `config/yaml/*.yaml` – Sensor-specific templates and multi-session presets.
- `scripts/convert_custom_bag_to_standard.py` – Utilities for normalizing data sources.
- `scripts/visualize_pose_graph.py` – Offline inspection of pose graph results.
- Papers / documentation referenced by Fast-LIO and Scan Context implementations.

## 12. Saved Data Outputs

Invoking `/save_map` (or running the `data_saver` routines) will populate `save_directory/sequence/` with:

- `pose_graph_3d_result.txt` – Optimized poses and covariance matrices (TUM-style).
- `data_time.txt` – Stage timing metrics (`t1–t5`) per keyframe.
- `key_point_frame/` – Downsampled keyframe point clouds for map reconstruction.
- `logs/` – Runtime logs and parameter snapshots (`runtime_params.yaml`).
- Optional ROS bag exports (`*_result.bag`) containing aligned point clouds and odometry.
- Optional `intensity/` imagery and `grid_map/` artifacts when enabled via `common/save_intensity_image` or `common/save_grid_map`.

These artifacts enable offline analysis, map stitching, or multi-session relocalization.

## 13. Script Highlights

Key helper scripts in `scripts/` include:

- `convert_custom_bag_to_standard.py` – Harmonizes heterogeneous rosbag topics into the expected naming conventions.
- `visualize_pose_graph.py` – Renders pose graphs and loop edges for quick diagnostics.
- `generate_gridmap.py` – Builds occupancy or intensity grid maps from saved keyframes.
- `compare_pointcloud_bags.py` – Compares coverage and density between bag recordings.
- `inspect_pointcloud_fields.py` – Validates point cloud channel consistency before processing.

Use these tools to accelerate data preparation, validation, and visualization in complex multi-session deployments.

## 14. License & Contributions

Please refer to the project’s license file. Contributions are welcome via pull requests; see the coding guidelines in the repository for style and testing expectations.
