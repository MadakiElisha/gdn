#!/usr/bin/env python3
"""Simulated barometer: /gdn/baro_meas = z_true + bias + noise (NED, positive down).
Decimated to ~10 Hz (realistic baro update rate)."""
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Float64

BIAS  = 5.0
SIGMA = 1.5
DT_US = 100_000   # 0.1 s -> 10 Hz

class BaroSim(Node):
    def __init__(self):
        super().__init__('gdn_baro_sim')
        self.pub = self.create_publisher(Float64, '/gdn/baro_meas', 10)
        self.sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.cb,
            qos_profile_sensor_data)
        self.rng = random.Random(11)
        self.last_ts = -10**9

    def cb(self, msg):
        if msg.timestamp - self.last_ts < DT_US:
            return
        self.last_ts = msg.timestamp
        m = Float64()
        m.data = msg.z + BIAS + self.rng.gauss(0.0, SIGMA)
        self.pub.publish(m)

def main():
    rclpy.init()
    node = BaroSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
