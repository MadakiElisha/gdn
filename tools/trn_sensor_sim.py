"""Simulated terrain sensors driven by bag truth, on a 0.5 s grid.

Publishes:
  /gdn/agl_meas  height above ground, m (R1.1 real-altimeter model, what the node
                 uses by default). The recorded flights were flown over flat ground,
                 so the map is placed underneath them: vehicle altitude AMSL =
                 H - z_true, where H is the highest map point within GDN_AGL_RADIUS
                 (default 1500 m) of the origin plus GDN_AGL_CLEARANCE (default 30 m).
  /gdn/trn_meas  terrain height directly (legacy "ideal" oracle, trn_model:=ideal)
  /gdn/truth     truth position for replay_scorer

Map: $GDN_MAP, else data/maps/terrain_db_sigma4.bin. The local-NED origin and
metres-per-degree come from the map header (grid centre), exactly as
gdn::MapDb::Load() does, so simulator and filter always agree on geometry.
"""
import os
import rclpy, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
from gdn_paths import load_map

class TrnSim(Node):
    def __init__(self):
        super().__init__('gdn_trn_sensor_sim')
        map_path = os.environ.get('GDN_MAP')  # None -> default sigma4 map
        self.alt, h = load_map(map_path)
        self.lat0, self.lon0, self.dlat, self.dlon = h['lat0'], h['lon0'], h['dlat'], h['dlon']
        self.lat_c, self.lon_c = h['lat_c'], h['lon_c']
        self.m_lat, self.m_lon = h['m_per_deg_lat'], h['m_per_deg_lon']
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.cb, qos)
        self.pub_m = self.create_publisher(Float64, '/gdn/trn_meas', 10)
        self.pub_a = self.create_publisher(Float64, '/gdn/agl_meas', 10)
        self.pub_t = self.create_publisher(PointStamped, '/gdn/truth', 10)
        self.last_k = -1; self.rng = np.random.default_rng(7)
        self.rng_a = np.random.default_rng(11)   # separate stream: trn_meas stays as before
        radius = float(os.environ.get('GDN_AGL_RADIUS', '1500'))
        clearance = float(os.environ.get('GDN_AGL_CLEARANCE', '30'))
        ny, nx = self.alt.shape
        ii, jj = np.mgrid[0:ny, 0:nx]
        north = (self.lat0 + ii * self.dlat - self.lat_c) * self.m_lat
        east = (self.lon0 + jj * self.dlon - self.lon_c) * self.m_lon
        near = np.hypot(north, east) <= radius
        self.H = float(self.alt[near].max() if near.any() else self.alt.max()) + clearance
        self.get_logger().info(f'TRN sensor-sim ready (0.5 s grid), map origin '
                               f'({self.lat_c:.6f}, {self.lon_c:.6f}), AGL datum H={self.H:.1f} m')

    def alt_at(self, x, y):
        fi = (self.lat_c + x/self.m_lat - self.lat0)/self.dlat
        fj = (self.lon_c + y/self.m_lon - self.lon0)/self.dlon
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
        h_t = self.alt_at(x, y)
        agl = (self.H - z) - h_t + self.rng_a.normal(0, 5) - self.rng_a.normal(0, 2)
        a = Float64(); a.data = float(agl); self.pub_a.publish(a)
        tp = PointStamped(); tp.header.stamp = self.get_clock().now().to_msg()
        tp.point.x, tp.point.y = float(x), float(y); self.pub_t.publish(tp)

rclpy.init(); rclpy.spin(TrnSim()); rclpy.shutdown()
