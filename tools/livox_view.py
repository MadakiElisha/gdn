#!/usr/bin/env python3
"""Show /livox/lidar (livox_ros_driver2/CustomMsg) in RViz, which can't display that type:
republish each frame as sensor_msgs/PointCloud2 on /livox/points (x, y, z, intensity =
reflectivity, line, offset_time), same frame and stamp, the no-return points left out.

    python3 tools/livox_view.py &
    rviz2 -d sim/livox_view.rviz           # the frame and the display already set
    python3 tools/livox_view.py --roll 0   # a Mid-360S mounted upright (the real one on a desk)

RViz draws only in frames the transform tree knows, and nothing publishes livox_frame
(the driver doesn't; a robot model or LIO would). So this also publishes one static
transform, livox_view -> livox_frame, rolled 180 degrees by default: GDN's simulated
Mid-360S hangs upside down (R2.4c), and with RViz's Fixed Frame on livox_view the ground
is below it.

Works on the simulated Mid-360S (R2.4c) and on the real driver alike. It needs
livox_ros_driver2's messages to subscribe (the dev image has them), and reads each frame
from its bytes (check_sim_sensors.parse_livox), which keeps up with 200,000 points a
second.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_sim_sensors import parse_livox  # noqa: E402

OUT = np.dtype({"names": ["x", "y", "z", "intensity", "line", "offset_time"],
                "formats": ["<f4", "<f4", "<f4", "<f4", "<f4", "<f4"], "offsets": [0, 4, 8, 12, 16, 20], "itemsize": 24})


class LivoxView(Node):
    def __init__(self, roll_deg, frame="livox_frame"):
        super().__init__("gdn_livox_view")
        self.tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "livox_view"
        t.child_frame_id = frame
        t.transform.rotation.x = math.sin(math.radians(roll_deg) / 2)
        t.transform.rotation.w = math.cos(math.radians(roll_deg) / 2)
        self.tf.sendTransform(t)
        self.pub = self.create_publisher(PointCloud2, "/livox/points", 5)
        self.sub = self.create_subscription(_custom_msg_type(), "/livox/lidar", self.on_frame, QoSProfile(depth=5),
                                            raw=True)
        self.n = 0
        self.get_logger().info(f"/livox/lidar -> /livox/points; livox_view -> {frame} rolled {roll_deg:g} deg "
                               "(RViz: rviz2 -d sim/livox_view.rviz)")

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
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--roll", type=float, default=180.0,
                    help="the sensor's roll in the level view frame, degrees (180: GDN's upside-down mount)")
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    try:
        rclpy.spin(LivoxView(a.roll))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
