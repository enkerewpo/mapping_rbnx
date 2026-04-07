---
name: slam_mapping
description: Build a 3D colored point cloud map of the environment using FAST-LIVO2 LiDAR-Inertial-Visual SLAM
---

# SLAM Mapping

Use this skill to build a 3D point cloud map of the robot's environment.

## When to use
- The robot needs to explore and map an unknown environment
- A new map is needed for a changed environment
- The operator requests "build a map" or "start mapping"

## How to use
1. Ensure the SLAM service is running (`robonix/sys/slam/status`)
2. Switch to mapping mode via `robonix/sys/slam/switch_mode` with mode="mapping"
3. Navigate the robot through the environment to cover the area
4. Save the map via `robonix/sys/slam/save_map` with a descriptive filename
5. The saved PCD file can later be loaded for localization

## Outputs
- Real-time odometry on `robonix/prm/base/odom`
- Colored 3D point cloud map saved to disk (PCD format)

## Notes
- Mapping quality depends on sensor coverage — slow, deliberate motion is better
- Avoid rapid rotation which can cause visual feature tracking loss
- The map is incrementally built; saving is non-destructive
