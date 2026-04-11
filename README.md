# mapping_rbnx

Robonix 3D LiDAR-Inertial SLAM system service, based on [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2).

Provides real-time odometry, incremental 3D mapping with loop closure, relocalization against pre-built maps, and 2D map projection for Nav2 integration. Pure CPU — no GPU/CUDA required.

Runs in Docker by default with automatic platform detection:
- **x86**: `ros:humble-ros-base` base image
- **Jetson Orin**: `dustynv/ros:humble-ros-base-l4t-r36.4.0` base image (JetPack/L4T compatible, ARM-optimized build flags)

Platform-specific config overrides tune parameters for each target (e.g. more aggressive downsampling on ARM).

## Architecture

```
robonix/prm/sensor/lidar3d ──┐
  (Livox CustomMsg)          │
                             ▼
robonix/prm/sensor/imu ───► FASTLIO2 lio_node ───► PGO / Localizer
  (sensor_msgs/Imu)          │                         │
                             │    ┌────────────────────┘
                             ▼    ▼
                     ┌──────────────────┐
                     │   Atlas Bridge   │──── gRPC ───► Robonix Atlas
                     └──────┬───────────┘
                            │
              ┌─────────────┼──────────────────┐
              ▼             ▼                  ▼
     prm/base/odom   srv/common/map/*    srv/slam/*
      (Odometry)    (PointCloud2,        (status, save,
                     OccupancyGrid,       load, switch,
                     LaserScan)           set_pose)
```

## SLAM Engine

4 ROS2 C++ nodes from `third_party/FASTLIO2_ROS2`:

| Node | Package | Description |
|------|---------|-------------|
| `lio_node` | `fastlio2` | Core LiDAR-Inertial odometry (ESKF + ikd-Tree) |
| `pgo_node` | `pgo` | Pose graph optimization + loop closure (GTSAM) |
| `localizer_node` | `localizer` | Two-stage coarse-to-fine ICP relocalization |
| `hba_node` | `hba` | Hierarchical Bundle Adjustment (large-scale map consistency) |

The `interface` package provides service definitions: `SaveMaps.srv`, `Relocalize.srv`, `IsValid.srv`, `RefineMap.srv`, `SavePoses.srv`.

## Contracts

### Provides

| Contract ID | Mode | ROS2 Topic | Message Type | Description |
|-------------|------|-----------|---------|-------------|
| `robonix/prm/base/odom` | topic_out | `/fastlio2/lio_odom` | `nav_msgs/Odometry` | 6-DoF real-time odometry |
| `robonix/srv/slam/status` | rpc | — | `slam/srv/GetSlamStatus` | SLAM status query (mode, hz, health) |
| `robonix/srv/slam/save_map` | rpc | `/pgo/save_maps` | `slam/srv/SaveMap` | Save 3D PCD map |
| `robonix/srv/slam/load_map` | rpc | `/localizer/relocalize` | `slam/srv/LoadMap` | Load map and start relocalization |
| `robonix/srv/slam/switch_mode` | rpc | — | `slam/srv/SwitchMode` | Switch mapping / localization / idle |
| `robonix/srv/slam/set_initial_pose` | rpc | `/localizer/relocalize` | `slam/srv/SetInitialPose` | Relocalization initial pose hint |
| `robonix/srv/common/map/pointcloud` | topic_out | `/fastlio2/world_cloud` | `sensor_msgs/PointCloud2` | Registered 3D point cloud in world frame |
| `robonix/srv/common/map/occupancy_grid` | topic_out | `/robonix/map/occupancy_grid` | `nav_msgs/OccupancyGrid` | 3D-to-2D projected occupancy grid (Nav2 static layer) |
| `robonix/srv/common/map/scan_2d` | topic_out | `/robonix/map/scan_2d` | `sensor_msgs/LaserScan` | 3D-to-2D projected laser scan (Nav2 obstacle layer) |

### Consumes

| Contract ID | Mode | Default Topic | Message Type | Description |
|-------------|------|-----------|---------|-------------|
| `robonix/prm/sensor/lidar3d` | topic_out | `/livox/lidar` | `livox_ros_driver2/CustomMsg` | 3D LiDAR point cloud input |
| `robonix/prm/sensor/imu` | topic_out | `/livox/imu` | `sensor_msgs/Imu` | IMU data input |

> The LiDAR driver is managed by a separate Robonix package (e.g. MID-360 driver), which registers the `robonix/prm/sensor/lidar3d` primitive interface. mapping_rbnx discovers and subscribes via Atlas.

### TF Tree

```
map ──(PGO/localizer)──► odom ──(LIO)──► base_link
                                              │
                                         lidar_frame
```

## ROS2 Topics (internal)

Full topic list published in mapping mode:

| Topic | Type | Source |
|-------|------|--------|
| `/fastlio2/lio_odom` | `nav_msgs/Odometry` | lio_node |
| `/fastlio2/lio_path` | `nav_msgs/Path` | lio_node |
| `/fastlio2/body_cloud` | `sensor_msgs/PointCloud2` | lio_node |
| `/fastlio2/world_cloud` | `sensor_msgs/PointCloud2` | lio_node |
| `/pgo/loop_markers` | `visualization_msgs/MarkerArray` | pgo_node |
| `/robonix/map/scan_2d` | `sensor_msgs/LaserScan` | pointcloud_to_laserscan |

ROS2 Services (provided by FASTLIO2_ROS2 nodes):

| Service | Type | Description |
|---------|------|-------------|
| `/pgo/save_maps` | `interface/srv/SaveMaps` | Save optimized map + patches |
| `/localizer/relocalize` | `interface/srv/Relocalize` | Start relocalization with PCD + initial pose |
| `/localizer/is_valid` | `interface/srv/IsValid` | Check relocalization convergence |
| `/hba/refine_map` | `interface/srv/RefineMap` | Large-scale map BA refinement |
| `/hba/save_poses` | `interface/srv/SavePoses` | Export optimized poses |

## Directory Structure

```
mapping_rbnx/
├── config/
│   ├── fastlio2_default.yaml          # Default config (LIO + PGO + Localizer + HBA)
│   └── platforms/
│       ├── jetson_orin.yaml           # Jetson Orin ARM-optimized overrides
│       └── x86_desktop.yaml           # x86 full-resolution overrides
├── docker/
│   ├── Dockerfile                     # x86 build (ros:humble-ros-base)
│   ├── Dockerfile.jetson              # Jetson build (L4T base, ARM flags)
│   ├── compose.yaml                   # Docker Compose base
│   ├── compose.jetson.yaml            # Jetson overlay
│   └── entrypoint.sh
├── launch/
│   ├── slam_mapping.launch.py         # Mapping: LIO + PGO + pointcloud_to_laserscan
│   └── slam_localization.launch.py    # Localization: LIO + Localizer + pointcloud_to_laserscan
├── scripts/
│   └── build.sh                       # Build script (Docker default, native optional)
├── skills/
│   ├── mapping/SKILL.md               # AI Agent "build map" skill
│   └── localization/SKILL.md          # AI Agent "localize" skill
├── src/mapping_rbnx/
│   ├── atlas_bridge.py                # gRPC <-> ROS2 bridge + Atlas registration
│   ├── __init__.py
│   └── __main__.py
├── third_party/
│   └── FASTLIO2_ROS2/                 # Git submodule (enkerewpo/FASTLIO2_ROS2)
├── robonix_manifest.yaml
└── README.md
```

## Usage

### Docker (default)

```bash
# Build
./scripts/build.sh

# Mapping
SLAM_MODE=mapping docker compose -f docker/compose.yaml up

# Save map (from another terminal)
docker exec mapping_rbnx_slam bash -c \
  '. /ws/install/setup.bash && ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: /maps/lab_map, save_patches: true}"'

# Localization
SLAM_MODE=localization docker compose -f docker/compose.yaml up
```

### Via Robonix

```bash
rbnx build -p .
rbnx start -p . --profile mapping
rbnx start -p . --profile localization
```

### Native (optional, requires ROS2 Humble on host)

```bash
RBNX_BUILD_MODE=native ./scripts/build.sh
SLAM_MODE=mapping ros2 launch mapping_rbnx slam_mapping.launch.py
```

## Configuration

Default config `config/fastlio2_default.yaml` contains 5 sections:

| Section | Description | Key Parameters |
|---------|-------------|----------------|
| `lio` | LIO core | `imu_topic`, `lidar_topic`, `ieskf_max_iter`, `map_resolution`, `det_range` |
| `pgo` | Loop closure | `loop_search_radius`, `loop_score_tresh`, `submap_resolution` |
| `localizer` | ICP relocalization | `update_hz`, `rough_score_thresh`, `refine_score_thresh` |
| `hba` | Map refinement | `window_size`, `voxel_size`, `hba_iter` |
| `robonix` | Bridge config | `odom_topic`, `cloud_topic`, `map_save_dir` |

## Integration Guide

### For navigation packages (Nav2)

mapping_rbnx provides two topics that Nav2 can consume directly:

| Nav2 Layer | Subscribe to | Type | Description |
|------------|-------------|------|-------------|
| Static map layer | `/robonix/map/occupancy_grid` | `nav_msgs/OccupancyGrid` | 2D occupancy grid projected from 3D SLAM map |
| Obstacle layer | `/robonix/map/scan_2d` | `sensor_msgs/LaserScan` | Real-time 2D scan projected from 3D point cloud |
| Odometry | `/fastlio2/lio_odom` | `nav_msgs/Odometry` | 6-DoF odometry for robot_localization / EKF |

Nav2 costmap config example:

```yaml
local_costmap:
  plugins: ["obstacle_layer", "inflation_layer"]
  obstacle_layer:
    plugin: "nav2_costmap_2d::ObstacleLayer"
    observation_sources: scan
    scan:
      topic: /robonix/map/scan_2d
      data_type: LaserScan
      marking: true
      clearing: true

global_costmap:
  plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
  static_layer:
    plugin: "nav2_costmap_2d::StaticLayer"
    map_topic: /robonix/map/occupancy_grid
```

TF tree published by mapping_rbnx:

```
map → odom → base_link → lidar_frame
```

Nav2 expects `map → odom` and `odom → base_link` transforms — both are provided.

### For LiDAR driver packages

mapping_rbnx consumes 3D LiDAR and IMU data. Your driver package should publish:

| Topic | Type | Required by |
|-------|------|-------------|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | fastlio2 `lio_node` |
| `/livox/imu` | `sensor_msgs/msg/Imu` | fastlio2 `lio_node` |

Topic names are configurable via `config/fastlio2_default.yaml` (`lio.lidar_topic`, `lio.imu_topic`).

If using Robonix Atlas, register your driver as a provider of `robonix/prm/sensor/lidar3d` and `robonix/prm/sensor/imu`. mapping_rbnx will discover topics via Atlas negotiation, falling back to the defaults above.

### For visualization / monitoring

| Topic | Type | Description |
|-------|------|-------------|
| `/fastlio2/world_cloud` | `sensor_msgs/PointCloud2` | Full registered 3D map (for RViz) |
| `/fastlio2/lio_path` | `nav_msgs/Path` | Trajectory trace |
| `/pgo/loop_markers` | `visualization_msgs/MarkerArray` | Detected loop closures |

### For Robonix Atlas consumers (gRPC)

The Atlas bridge exposes all contracts via gRPC on port `50120` (configurable via `MAPPING_GRPC_PORT`):

```python
import grpc
import robonix_contracts_pb2_grpc as contracts

channel = grpc.insecure_channel("localhost:50120")

# Stream odometry
stub = contracts.PrmBaseOdomStub(channel)
for odom in stub.Stream(empty_pb2.Empty()):
    print(f"pose: {odom.pose.pose.position}")

# Query SLAM status
stub = contracts.SysSlamStatusStub(channel)
status = stub.Call(slam_pb2.GetSlamStatus_Request())
print(f"mode={status.status.mode} hz={status.status.odom_hz}")

# Save map
stub = contracts.SysSlamSaveMapStub(channel)
resp = stub.Call(slam_pb2.SaveMap_Request(filename="my_map"))

# Load map + relocalize
stub = contracts.SysSlamLoadMapStub(channel)
resp = stub.Call(slam_pb2.LoadMap_Request(path="/maps/my_map/GlobalMap.pcd"))
```

### For AI agent skills

Two skills are registered with Atlas for the Robonix Pilot/Executor to invoke:

- **`slam_mapping`** — "Build a 3D map of the environment" (see `skills/mapping/SKILL.md`)
- **`slam_localization`** — "Localize in a pre-built map" (see `skills/localization/SKILL.md`)

These skills describe when and how to call the SLAM service contracts, enabling the reasoning engine to autonomously trigger mapping or localization workflows.

## Dependencies

All dependencies are handled inside the Docker container. For native builds:

- ROS2 Humble
- PCL, Eigen3, Sophus (header-only >= 1.22), GTSAM >= 4.2, yaml-cpp
- Livox SDK2 + livox_ros_driver2 (CustomMsg definitions)
- ros-humble-pointcloud-to-laserscan, ros-humble-rmw-cyclonedds-cpp
- grpcio, protobuf, numpy, pyyaml (Atlas bridge)

## License

MulanPSL-2.0
