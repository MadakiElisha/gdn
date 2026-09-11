import rclpy, math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

class Scorer(Node):
    def __init__(self):
        super().__init__('gdn_replay_scorer')
        self.odom = None; self.t0 = None; self.ts = []; self.errs = []; self.lp = 0.0
        self.create_subscription(Odometry, '/gdn/odom', self.cb_o, 10)
        self.create_subscription(PointStamped, '/gdn/truth', self.cb_t, 10)
        self.get_logger().info('replay scorer alive and subscribed')
    def cb_o(self, m): self.odom = m
    def cb_t(self, m):
        if self.odom is None: return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None: self.t0 = t
        e = math.hypot(m.point.x - self.odom.pose.pose.position.x,
                       m.point.y - self.odom.pose.pose.position.y)
        self.ts.append(t - self.t0); self.errs.append(e)
        if t - self.lp > 20:
            self.lp = t
            self.get_logger().info(f't={t - self.t0:6.1f} s  horiz err {e:8.1f} m')

rclpy.init(); n = Scorer()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    ts = np.array(n.ts); e = np.array(n.errs); w = (ts > 60) & (ts < 300)
    if w.any():
        print(f'RMS 60-300s: {np.sqrt((e[w]**2).mean()):.1f} m | MAX 60-300s: {e[w].max():.1f} m')
rclpy.shutdown()
