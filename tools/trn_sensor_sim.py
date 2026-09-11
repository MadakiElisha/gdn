import rclpy, numpy as np, struct
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
from pathlib import Path

LAT_C, LON_C = 47.3977, 8.5456
M_LAT = 111320.0; M_LON = 111320.0*np.cos(np.radians(LAT_C))

class TrnSim(Node):
    def __init__(self):
        super().__init__('gdn_trn_sensor_sim')
        b = (Path.home()/'gdn_workspace/data/maps/terrain_db.bin').read_bytes()
        nx, ny, self.lat0, self.lon0, self.dlat, self.dlon = struct.unpack('<iidddd', b[:40])
        self.alt = np.frombuffer(b[40:], dtype='<f4').reshape(ny, nx)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.cb, qos)
        self.pub_m = self.create_publisher(Float64, '/gdn/trn_meas', 10)
        self.pub_t = self.create_publisher(PointStamped, '/gdn/truth', 10)
        self.last_k = -1; self.rng = np.random.default_rng(7)
        self.get_logger().info('TRN sensor-sim ready (0.5 s timestamp grid)')

    def alt_at(self, x, y):
        fi = (LAT_C + x/M_LAT - self.lat0)/self.dlat
        fj = (LON_C + y/M_LON - self.lon0)/self.dlon
        fi = min(max(fi, 0.0), self.alt.shape[0]-1.001)
        fj = min(max(fj, 0.0), self.alt.shape[1]-1.001)
        i0, j0 = int(fi), int(fj); ti, tj = fi-i0, fj-j0
        return float((1-ti)*((1-tj)*self.alt[i0,j0] + tj*self.alt[i0,j0+1]) +
                     ti*((1-tj)*self.alt[i0+1,j0] + tj*self.alt[i0+1,j0+1]))

    def cb(self, msg):
        t = msg.timestamp * 1e-6
        k = int(np.floor(t * 2.0))                 # fire exactly once per 0.5 s bag-time cell
        if k == self.last_k: return
        self.last_k = k
        x, y, z = msg.x, msg.y, msg.z
        h_agl = max(-z, 0.0)
        n5 = self.rng.normal(0, 5); n2 = self.rng.normal(0, 2)
        meas = (self.alt_at(x, y) + h_agl + n5) - (h_agl + n2)
        m = Float64(); m.data = float(meas); self.pub_m.publish(m)
        tp = PointStamped(); tp.header.stamp = self.get_clock().now().to_msg()
        tp.point.x, tp.point.y = float(x), float(y); self.pub_t.publish(tp)

rclpy.init(); rclpy.spin(TrnSim()); rclpy.shutdown()
