#!/usr/bin/env python3
# Voxel-accumulate world_cloud into a persistent downsampled point cloud.
# Publishes /robonix/map/cloud_accumulated at 1 Hz.
#
# Each voxel key (i, j, k) stores:
#   - hit_count  : how many frames have observed this voxel (capped)
#   - last_frame : last frame index the voxel was observed
#   - xyz        : the first-observed coordinate (cheap, no recomputation)
#
# Two policies gated by params:
#   max_points_cap  — hard FIFO cap on voxels kept (default 300k).
#   stale_frames    — if >0, voxels not observed for N frames AND with
#                     hit_count below confirm_hits are pruned. This lets
#                     transient objects (people, moved chairs) fade, while
#                     repeatedly-observed structure (walls) stays.
#
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class CloudAcc(Node):
    def __init__(self):
        super().__init__('cloud_accumulator')
        self.voxel = 0.1          # m
        self.frame = 'lidar'
        self.max_points_cap = 300_000
        # Transient decay:
        #   stale_frames = 0 disables decay (old behavior: add-only).
        #   Otherwise voxels not re-observed within N frames AND with fewer
        #   than `confirm_hits` hits get pruned on each publish tick.
        self.stale_frames = 300     # ~5 min at 1Hz cloud stream ÷ 1s publish
        self.confirm_hits = 3       # hits needed to be considered "static"
        self.hit_cap = 50           # don't let a single voxel grow unboundedly

        # voxel_key -> [hit_count, last_frame_idx, x, y, z]
        self.voxels: dict[tuple[int, int, int], list] = {}
        self.frame_idx = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(PointCloud2, '/robonix/map/cloud_accumulated', latched)
        self.create_subscription(PointCloud2, '/fastlio2/world_cloud', self.cb, 10)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(
            f'CloudAcc: voxel={self.voxel}m, cap={self.max_points_cap}, '
            f'stale_frames={self.stale_frames}, confirm_hits={self.confirm_hits}'
        )

    def cb(self, msg):
        self.frame_idx += 1
        offs = {f.name: f.offset for f in msg.fields}
        ox, oy, oz = offs['x'], offs['y'], offs['z']
        ps = msg.point_step
        n_pts = msg.width * msg.height
        if n_pts == 0:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)[:n_pts * ps].reshape(n_pts, ps)
        x = arr[:, ox:ox + 4].copy().view(np.float32).ravel()
        y = arr[:, oy:oy + 4].copy().view(np.float32).ravel()
        z = arr[:, oz:oz + 4].copy().view(np.float32).ravel()
        mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x = x[mask]; y = y[mask]; z = z[mask]
        if x.size == 0:
            return

        ix = np.floor(x / self.voxel).astype(np.int32)
        iy = np.floor(y / self.voxel).astype(np.int32)
        iz = np.floor(z / self.voxel).astype(np.int32)
        for i in range(len(x)):
            k = (int(ix[i]), int(iy[i]), int(iz[i]))
            v = self.voxels.get(k)
            if v is None:
                self.voxels[k] = [1, self.frame_idx, float(x[i]), float(y[i]), float(z[i])]
            else:
                # bump hit count (capped) and refresh last_frame
                v[0] = min(self.hit_cap, v[0] + 1)
                v[1] = self.frame_idx

        # Hard FIFO cap — evict least-recently-seen.
        if len(self.voxels) > self.max_points_cap:
            # Sort by last_frame ascending, drop the oldest to fit.
            items = sorted(self.voxels.items(), key=lambda kv: kv[1][1])
            keep_from = len(items) - self.max_points_cap
            for kk, _ in items[:keep_from]:
                del self.voxels[kk]

    def _decay_sweep(self):
        """Prune transient voxels: not seen for `stale_frames` AND hits < confirm_hits."""
        if self.stale_frames <= 0:
            return
        cutoff = self.frame_idx - self.stale_frames
        stale = [k for k, v in self.voxels.items()
                 if v[1] < cutoff and v[0] < self.confirm_hits]
        for k in stale:
            del self.voxels[k]
        if stale:
            self.get_logger().debug(f'pruned {len(stale)} transient voxels')

    def publish(self):
        self._decay_sweep()
        if not self.voxels:
            return
        pts = np.array(
            [[v[2], v[3], v[4]] for v in self.voxels.values()],
            dtype=np.float32,
        )
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.height = 1
        msg.width = pts.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = pts.tobytes()
        msg.is_dense = True
        self.pub.publish(msg)


def main():
    rclpy.init()
    n = CloudAcc()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
