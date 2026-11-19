#!/usr/bin/env python3
"""Show /livox/lidar (livox_ros_driver2/CustomMsg) in RViz, which can't display that type:
republish each frame as sensor_msgs/PointCloud2 on /livox/points (x, y, z, intensity =
reflectivity, line, offset_time), same frame and stamp, the no-return points left out.

    python3 tools/livox_view.py            # then in RViz: Add > By topic > /livox/points
                                           # Fixed Frame: livox_frame (no TF to the world)

Works on the simulated Mid-360S (R2.4c) and on the real driver alike. It reads the
message bytes directly (check_sim_sensors.parse_livox), so it needs no Livox package
and keeps up with 200,000 points a second.
"""
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2, PointField

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_sim_sensors import parse_livox  # noqa: E402

OUT = np.dtype({"names": ["x", "y", "z", "intensity", "line", "offset_time"],
                "formats": ["<f4", "<f4", "<f4", "<f4", "<f4", "<f4"], "offsets": [0, 4, 8, 12, 16, 20], "itemsize": 24})


class LivoxView(Node):
    def __init__(self):
        super().__init__("gdn_livox_view")
        self.pub = self.create_publisher(PointCloud2, "/livox/points", 5)
        self.sub = self.create_subscription(_custom_msg_type(), "/livox/lidar", self.on_frame, QoSProfile(depth=5),
                                            raw=True)
        self.n = 0
        self.get_logger().info("/livox/lidar -> /livox/points (RViz: Fixed Frame livox_frame)")

    def on_frame(self, data):
        st, frame, _, _, _, p = parse_livox(data)
        x, y, z = p["x"], p["y"], p["z"]
        hit = (x != 0) | (y != 0) | (z != 0)
        q = np.zeros(int(hit.sum()), OUT)
        for k in ("x", "y", "z"):
            q[k] = p[k][hit]
        q["intensity"] = p["reflectivity"][hit]
        q["line"] = p["line"][hit]
        q["offset_time"] = p["offset_time"][hit] * 1e-6        # ms
        m = PointCloud2()
        m.header.stamp.sec = int(st)
        m.header.stamp.nanosec = int(round((st - int(st)) * 1e9)) % 1_000_000_000
        m.header.frame_id = frame
        m.height, m.width = 1, len(q)
        m.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                    for n, o in zip(OUT.names, (0, 4, 8, 12, 16, 20))]
        m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 24, 24 * len(q), True
        m.data = q.tobytes()
        self.pub.publish(m)
        self.n += 1
        if self.n % 50 == 1:
            self.get_logger().info(f"frame {self.n}: {len(q)} returns of {len(p)} points")


def _custom_msg_type():
    try:
        from livox_ros_driver2.msg import CustomMsg
        return CustomMsg
    except ImportError:
        sys.exit("livox_view: livox_ros_driver2's messages aren't sourced (the dev image has them; "
                 "on the host, source ~/ws_livox/install/setup.bash)")


def main():
    rclpy.init()
    try:
        rclpy.spin(LivoxView())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
