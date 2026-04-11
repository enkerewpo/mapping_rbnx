---
name: slam_mapping
description: Build a 3D point cloud map of the environment using FASTLIO2 LiDAR-Inertial SLAM with PGO loop closure
---

# SLAM Mapping

Use this skill to build a 3D point cloud map of the robot's environment.

## When to use
- The robot needs to explore and map an unknown environment
- A new map is needed for a changed environment
- The operator requests "build a map" or "start mapping"

## How to use
1. Ensure the SLAM service is running (`robonix/srv/slam/status`)
2. Switch to mapping mode via `robonix/srv/slam/switch_mode` with mode="mapping"
3. Navigate the robot through the environment to cover the area
4. Save the map via `robonix/srv/slam/save_map` with a descriptive filename
5. Optionally refine map consistency via HBA (`/hba/refine_map`)

## Outputs
- Real-time odometry on `robonix/prm/base/odom` (from `/fastlio2/lio_odom`)
- Loop-closure-corrected trajectory (from PGO)
- 3D point cloud map saved to disk (PCD format, via `/pgo/save_maps`)

## Notes
- Mapping quality depends on sensor coverage — slow, deliberate motion is better
- PGO automatically detects loop closures and corrects drift
- The map is incrementally built; saving is non-destructive
- For large-scale maps, use HBA post-processing for global consistency
