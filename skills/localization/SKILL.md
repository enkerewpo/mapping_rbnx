---
name: slam_localization
description: Localize the robot within a pre-built point cloud map using FASTLIO2 + two-stage ICP relocalization
---

# SLAM Localization

Use this skill to localize the robot within a previously built map.

## When to use
- The robot is deployed in a known, previously mapped environment
- Navigation requires accurate global pose estimation
- The operator requests "localize" or "use existing map"

## How to use
1. Ensure a pre-built map exists (PCD file from a mapping session)
2. Load the map via `robonix/srv/slam/load_map` with the map file path
3. Optionally set an initial pose hint via `robonix/srv/slam/set_initial_pose`
4. The system will output corrected odometry on `robonix/prm/base/odom`

## Outputs
- Drift-corrected 6-DoF pose via `robonix/prm/base/odom`
- Localization validity check via `/localizer/is_valid`

## Notes
- Relocalization uses two-stage coarse-to-fine ICP for robust convergence
- Initial convergence may take a few seconds after loading the map
- If localization drifts, set a new initial pose hint to re-trigger ICP
- The environment should not have changed significantly from when the map was built
