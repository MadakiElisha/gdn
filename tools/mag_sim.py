#!/usr/bin/env python3
"""Simulated magnetometer (Pure Python, no scipy/numpy dependencies).
/gdn/mag_meas = R^T * m_ref + b_true + noise"""
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleAttitude
from geometry_msgs.msg import Vector3

# WMM field for 47.398N, 8.546E (PX4 SITL spawn, Zurich)
# Declination=+3.5deg, Inclination=+63deg, Total=47.5uT
M_REF = [21.5, 1.3, 42.3]  # uT (N, E, D)
B_TRUE = [2.0, -1.0, 1.0]  # uT (hard-iron)
SIGMA = 2.0                # uT (noise, sigma=2)
DT_US = 50_000             # 0.05 s -> 20 Hz

def quat_to_rot(q_w, q_x, q_y, q_z):
    xx, yy, zz = q_x*q_x, q_y*q_y, q_z*q_z
    xy, xz, yz = q_x*q_y, q_x*q_z, q_y*q_z
    wx, wy, wz = q_w*q_x, q_w*q_y, q_w*q_z
    return [
        [1 - 2*(yy + zz), 2*(xy - wz), 2*(xz + wy)],
        [2*(xy + wz), 1 - 2*(xx + zz), 2*(yz - wx)],
        [2*(xz - wy), 2*(yz + wx), 1 - 2*(xx + yy)]
    ]

class MagSim(Node):
    def __init__(self):
        super().__init__('gdn_mag_sim')
        self.pub = self.create_publisher(Vector3, '/gdn/mag_meas', 10)
        self.sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.cb,
            qos_profile_sensor_data)
        self.last_ts = -10**9
        self.rng = random.Random(42)

    def cb(self, msg):
        if msg.timestamp - self.last_ts < DT_US:
            return
        self.last_ts = msg.timestamp
        
        # PX4 vehicle_attitude.q is body-to-NED quaternion (w, x, y, z)
        q_w, q_x, q_y, q_z = float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])
        R = quat_to_rot(q_w, q_x, q_y, q_z)
        
        # Body frame field = R^T * m_ref
        m_body = [
            R[0][0]*M_REF[0] + R[1][0]*M_REF[1] + R[2][0]*M_REF[2],
            R[0][1]*M_REF[0] + R[1][1]*M_REF[1] + R[2][1]*M_REF[2],
            R[0][2]*M_REF[0] + R[1][2]*M_REF[1] + R[2][2]*M_REF[2]
        ]
        
        out = Vector3()
        out.x = float(m_body[0] + B_TRUE[0] + self.rng.gauss(0, SIGMA))
        out.y = float(m_body[1] + B_TRUE[1] + self.rng.gauss(0, SIGMA))
        out.z = float(m_body[2] + B_TRUE[2] + self.rng.gauss(0, SIGMA))
        self.pub.publish(out)

def main():
    rclpy.init()
    node = MagSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
