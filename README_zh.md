# mapping_rbnx

基于 [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2) 的 Robonix 3D 激光惯性 SLAM 系统服务。

提供实时里程计、带回环检测的增量式 3D 建图、基于已有地图的重定位，以及面向 Nav2 的 2D 地图投影。纯 CPU 运算，不需要 GPU/CUDA。

默认在 Docker 容器中运行，自动检测平台：
- **x86**：`ros:humble-ros-base` 基础镜像
- **Jetson Orin**：`dustynv/ros:humble-ros-base-l4t-r36.4.0` 基础镜像（JetPack/L4T 兼容，ARM 优化编译）

平台配置 override 会针对不同硬件调优参数（如 ARM 上更激进的降采样）。

## 架构

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

## SLAM 引擎

底层由 4 个 ROS2 C++ 节点组成（来自 `third_party/FASTLIO2_ROS2`）：

| 节点 | 包名 | 功能 |
|------|------|------|
| `lio_node` | `fastlio2` | 核心激光惯性里程计（ESKF + ikd-Tree） |
| `pgo_node` | `pgo` | 位姿图优化 + 回环检测（GTSAM） |
| `localizer_node` | `localizer` | 两阶段 coarse-to-fine ICP 重定位 |
| `hba_node` | `hba` | 层次化 Bundle Adjustment（大场景地图一致性优化） |

消息定义包 `interface` 提供 `SaveMaps.srv`、`Relocalize.srv`、`IsValid.srv`、`RefineMap.srv`、`SavePoses.srv`。

## 契约 (Contracts)

### 提供 (Provides)

| 契约 ID | 模式 | ROS2 Topic | 消息类型 | 说明 |
|---------|------|-----------|---------|------|
| `robonix/prm/base/odom` | topic_out | `/fastlio2/lio_odom` | `nav_msgs/Odometry` | 6-DoF 实时里程计 |
| `robonix/srv/slam/status` | rpc | — | `slam/srv/GetSlamStatus` | SLAM 状态查询（模式、频率、健康） |
| `robonix/srv/slam/save_map` | rpc | `/pgo/save_maps` | `slam/srv/SaveMap` | 保存 3D PCD 地图 |
| `robonix/srv/slam/load_map` | rpc | `/localizer/relocalize` | `slam/srv/LoadMap` | 加载地图并启动重定位 |
| `robonix/srv/slam/switch_mode` | rpc | — | `slam/srv/SwitchMode` | 切换 mapping / localization / idle |
| `robonix/srv/slam/set_initial_pose` | rpc | `/localizer/relocalize` | `slam/srv/SetInitialPose` | 重定位初始位姿提示 |
| `robonix/srv/common/map/pointcloud` | topic_out | `/fastlio2/world_cloud` | `sensor_msgs/PointCloud2` | 世界坐标系下的 3D 注册点云 |
| `robonix/srv/common/map/occupancy_grid` | topic_out | `/robonix/map/occupancy_grid` | `nav_msgs/OccupancyGrid` | 3D 投影 2D 栅格地图（Nav2 静态层） |
| `robonix/srv/common/map/scan_2d` | topic_out | `/robonix/map/scan_2d` | `sensor_msgs/LaserScan` | 3D 投影 2D 扫描（Nav2 障碍物层） |

### 消费 (Consumes)

| 契约 ID | 模式 | 默认 Topic | 消息类型 | 说明 |
|---------|------|-----------|---------|------|
| `robonix/prm/sensor/lidar3d` | topic_out | `/livox/lidar` | `livox_ros_driver2/CustomMsg` | 3D 激光雷达点云输入 |
| `robonix/prm/sensor/imu` | topic_out | `/livox/imu` | `sensor_msgs/Imu` | IMU 数据输入 |

> LiDAR 驱动由独立的 Robonix package 管理（如 MID-360 驱动包），注册 `robonix/prm/sensor/lidar3d` 原语接口。mapping_rbnx 通过 Atlas 发现并订阅。

### TF 树

```
map ──(PGO/localizer)──► odom ──(LIO)──► base_link
                                              │
                                         lidar_frame
```

## ROS2 Topics（内部）

mapping 模式下发布的完整 topic 列表：

| Topic | 类型 | 来源 |
|-------|------|------|
| `/fastlio2/lio_odom` | `nav_msgs/Odometry` | lio_node |
| `/fastlio2/lio_path` | `nav_msgs/Path` | lio_node |
| `/fastlio2/body_cloud` | `sensor_msgs/PointCloud2` | lio_node |
| `/fastlio2/world_cloud` | `sensor_msgs/PointCloud2` | lio_node |
| `/pgo/loop_markers` | `visualization_msgs/MarkerArray` | pgo_node |
| `/robonix/map/scan_2d` | `sensor_msgs/LaserScan` | pointcloud_to_laserscan |

ROS2 Service（由 FASTLIO2_ROS2 节点提供）：

| Service | 类型 | 说明 |
|---------|------|------|
| `/pgo/save_maps` | `interface/srv/SaveMaps` | 保存优化后的地图 + patches |
| `/localizer/relocalize` | `interface/srv/Relocalize` | 给定 PCD + 初始位姿启动重定位 |
| `/localizer/is_valid` | `interface/srv/IsValid` | 检查重定位是否收敛 |
| `/hba/refine_map` | `interface/srv/RefineMap` | 大场景地图 BA 精修 |
| `/hba/save_poses` | `interface/srv/SavePoses` | 导出优化后的位姿 |

## 目录结构

```
mapping_rbnx/
├── config/
│   ├── fastlio2_default.yaml          # 默认配置（LIO + PGO + Localizer + HBA）
│   └── platforms/
│       ├── jetson_orin.yaml           # Jetson Orin ARM 优化参数
│       └── x86_desktop.yaml           # x86 高精度参数
├── docker/
│   ├── Dockerfile                     # x86 构建（ros:humble-ros-base）
│   ├── Dockerfile.jetson              # Jetson 构建（L4T 基础镜像，ARM 编译参数）
│   ├── compose.yaml                   # Docker Compose 基础配置
│   ├── compose.jetson.yaml            # Jetson overlay
│   └── entrypoint.sh
├── launch/
│   ├── slam_mapping.launch.py         # 建图模式：LIO + PGO + pointcloud_to_laserscan
│   └── slam_localization.launch.py    # 定位模式：LIO + Localizer + pointcloud_to_laserscan
├── scripts/
│   └── build.sh                       # 构建脚本（默认 Docker，可选原生）
├── skills/
│   ├── mapping/SKILL.md               # AI Agent「建图」技能
│   └── localization/SKILL.md          # AI Agent「定位」技能
├── src/mapping_rbnx/
│   ├── atlas_bridge.py                # gRPC <-> ROS2 桥接 + Atlas 注册
│   ├── __init__.py
│   └── __main__.py
├── third_party/
│   └── FASTLIO2_ROS2/                 # Git submodule (enkerewpo/FASTLIO2_ROS2)
├── robonix_manifest.yaml
└── README.md
```

## 使用方式

### Docker（默认）

```bash
# 构建
./scripts/build.sh

# 建图
SLAM_MODE=mapping docker compose -f docker/compose.yaml up

# 保存地图（另一个终端）
docker exec mapping_rbnx_slam bash -c \
  '. /ws/install/setup.bash && ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: /maps/lab_map, save_patches: true}"'

# 定位
SLAM_MODE=localization docker compose -f docker/compose.yaml up
```

### 通过 Robonix

```bash
rbnx build -p .
rbnx start -p . --profile mapping
rbnx start -p . --profile localization
```

### 原生部署（可选，需要宿主机安装 ROS2 Humble）

```bash
RBNX_BUILD_MODE=native ./scripts/build.sh
SLAM_MODE=mapping ros2 launch mapping_rbnx slam_mapping.launch.py
```

## 配置

默认配置 `config/fastlio2_default.yaml` 包含 5 个段：

| 段 | 说明 | 关键参数 |
|---|------|---------|
| `lio` | LIO 核心 | `imu_topic`, `lidar_topic`, `ieskf_max_iter`, `map_resolution`, `det_range` |
| `pgo` | 回环检测 | `loop_search_radius`, `loop_score_tresh`, `submap_resolution` |
| `localizer` | ICP 重定位 | `update_hz`, `rough_score_thresh`, `refine_score_thresh` |
| `hba` | 地图精修 | `window_size`, `voxel_size`, `hba_iter` |
| `robonix` | Bridge 配置 | `odom_topic`, `cloud_topic`, `map_save_dir` |

## 集成指南

### 导航包（Nav2）接入

mapping_rbnx 提供两个可被 Nav2 直接消费的 topic：

| Nav2 层 | 订阅 Topic | 类型 | 说明 |
|---------|-----------|------|------|
| 静态地图层 | `/robonix/map/occupancy_grid` | `nav_msgs/OccupancyGrid` | 3D SLAM 地图投影的 2D 栅格地图 |
| 障碍物层 | `/robonix/map/scan_2d` | `sensor_msgs/LaserScan` | 3D 点云实时投影的 2D 扫描 |
| 里程计 | `/fastlio2/lio_odom` | `nav_msgs/Odometry` | 6-DoF 里程计，供 robot_localization / EKF 使用 |

Nav2 costmap 配置示例：

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

mapping_rbnx 发布的 TF 树：

```
map → odom → base_link → lidar_frame
```

Nav2 需要 `map → odom` 和 `odom → base_link` 变换，两者均由 mapping_rbnx 提供。

### LiDAR 驱动包接入

mapping_rbnx 消费 3D LiDAR 和 IMU 数据，驱动包需要发布：

| Topic | 类型 | 被谁消费 |
|-------|------|---------|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | fastlio2 `lio_node` |
| `/livox/imu` | `sensor_msgs/msg/Imu` | fastlio2 `lio_node` |

Topic 名称可通过 `config/fastlio2_default.yaml` 配置（`lio.lidar_topic`、`lio.imu_topic`）。

如果使用 Robonix Atlas，驱动包应注册为 `robonix/prm/sensor/lidar3d` 和 `robonix/prm/sensor/imu` 的 provider。mapping_rbnx 会通过 Atlas 协商发现 topic，找不到时回退到上述默认值。

### 可视化 / 监控

| Topic | 类型 | 说明 |
|-------|------|------|
| `/fastlio2/world_cloud` | `sensor_msgs/PointCloud2` | 完整的 3D 注册地图（用于 RViz） |
| `/fastlio2/lio_path` | `nav_msgs/Path` | 轨迹 |
| `/pgo/loop_markers` | `visualization_msgs/MarkerArray` | 检测到的回环 |

### Robonix Atlas 消费者（gRPC）

Atlas bridge 在端口 `50120`（可通过 `MAPPING_GRPC_PORT` 配置）暴露所有契约的 gRPC 接口：

```python
import grpc
import robonix_contracts_pb2_grpc as contracts

channel = grpc.insecure_channel("localhost:50120")

# 流式获取里程计
stub = contracts.PrmBaseOdomStub(channel)
for odom in stub.Stream(empty_pb2.Empty()):
    print(f"pose: {odom.pose.pose.position}")

# 查询 SLAM 状态
stub = contracts.SysSlamStatusStub(channel)
status = stub.Call(slam_pb2.GetSlamStatus_Request())
print(f"mode={status.status.mode} hz={status.status.odom_hz}")

# 保存地图
stub = contracts.SysSlamSaveMapStub(channel)
resp = stub.Call(slam_pb2.SaveMap_Request(filename="my_map"))

# 加载地图 + 重定位
stub = contracts.SysSlamLoadMapStub(channel)
resp = stub.Call(slam_pb2.LoadMap_Request(path="/maps/my_map/GlobalMap.pcd"))
```

### AI Agent 技能

两个技能已注册到 Atlas，供 Robonix Pilot/Executor 调用：

- **`slam_mapping`** — 「建立环境的 3D 地图」（见 `skills/mapping/SKILL.md`）
- **`slam_localization`** — 「在已有地图中定位」（见 `skills/localization/SKILL.md`）

这些技能描述了何时以及如何调用 SLAM 服务契约，使推理引擎能够自主触发建图或定位工作流。

## 依赖

所有依赖在 Docker 容器内自动处理。原生构建需要：

- ROS2 Humble
- PCL, Eigen3, Sophus (header-only >= 1.22), GTSAM >= 4.2, yaml-cpp
- Livox SDK2 + livox_ros_driver2（CustomMsg 消息定义）
- ros-humble-pointcloud-to-laserscan, ros-humble-rmw-cyclonedds-cpp
- grpcio, protobuf, numpy, pyyaml（Atlas bridge）

## 许可证

MulanPSL-2.0
