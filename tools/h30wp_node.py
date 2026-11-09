#!/usr/bin/env python3
"""ROS 2 driver for the Yesense / WHEELTEC H30WP IMU (serial, YESENSE protocol).

Publishes, in the IMU's own front-left-up axes (as printed on the lid, which is also
the ROS convention, REP-103):
  /imu/data         sensor_msgs/Imu            acceleration m/s^2, angular rate rad/s
                                               (orientation from the IMU if it sends one)
  /imu/mag          sensor_msgs/MagneticField  tesla (from the raw field, mGauss)
  /imu/temperature  sensor_msgs/Temperature    deg C

    python3 tools/h30wp_node.py                               # /dev/gdn_imu or the only ttyUSB
    python3 tools/h30wp_node.py --ros-args -p port:=/dev/ttyUSB0 -p baud:=460800

Time stamps: if the IMU sends its sample timestamp (DATA ID 0x51, microseconds), each
message is stamped with the host time of the first sample plus the IMU's own elapsed
time. That keeps the spacing as even as the IMU's clock instead of USB/serial jitter,
which matters for the Allan fit. Without it, the host receive time is used. Lost frames
(gaps in the frame number) are counted and logged.

Needs pyserial (sudo apt install python3-serial). Decoding is shared with
tools/h30wp_check.py, which has a self-test against the protocol document's example.
"""
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from builtin_interfaces.msg import Time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from h30wp_check import Parser, find_ports, self_test  # noqa: E402

DEG = math.pi / 180.0


class H30wpNode(Node):
    def __init__(self):
        super().__init__("h30wp")
        default_port = "/dev/gdn_imu" if os.path.exists("/dev/gdn_imu") else (find_ports() or ["/dev/ttyUSB0"])[0]
        self.port = self.declare_parameter("port", default_port).value
        self.baud = int(self.declare_parameter("baud", 460800).value)
        self.frame_id = self.declare_parameter("frame_id", "imu_link").value
        self.pub_imu = self.create_publisher(Imu, "/imu/data", 200)
        self.pub_mag = self.create_publisher(MagneticField, "/imu/mag", 50)
        self.pub_tmp = self.create_publisher(Temperature, "/imu/temperature", 10)
        self.ser = None
        self.open_port()
        self.parser = Parser()
        self.last_seq = None
        self.lost = 0
        self.n = 0
        self.t0_host = None          # host time of the first stamped sample, ns
        self.ts0 = None              # IMU sample timestamp of that sample, us (unwrapped)
        self.ts_prev = None
        self.ts_wraps = 0
        self.t_report = time.time()
        self.create_timer(0.002, self.poll)
        self.get_logger().info(f"H30WP on {self.port} at {self.baud} baud, frame {self.frame_id}")

    def open_port(self):
        import serial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.ser.reset_input_buffer()
            return True
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().error(f"can't open {self.port}: {e}", throttle_duration_sec=5.0)
            return False

    def stamp(self, d):
        now_ns = self.get_clock().now().nanoseconds
        if 0x51 not in d:
            return now_ns
        ts = d[0x51]
        if self.ts_prev is not None and ts < self.ts_prev:
            self.ts_wraps += 1                                     # uint32 microseconds wrap every 71.6 min
        self.ts_prev = ts
        ts_u = ts + self.ts_wraps * (1 << 32)
        if self.t0_host is None:
            self.t0_host, self.ts0 = now_ns, ts_u
        return self.t0_host + (ts_u - self.ts0) * 1000

    def poll(self):
        import serial
        if self.ser is None:
            if time.time() - getattr(self, "_t_retry", 0.0) < 1.0 or not self.open_port():
                self._t_retry = time.time()
                return
            self.get_logger().info(f"{self.port} open again")
            self.t0_host = self.ts0 = self.ts_prev = self.last_seq = None   # new session: restart the clock mapping
            self.ts_wraps = 0
        try:
            chunk = self.ser.read(self.ser.in_waiting or 1)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"IMU disconnected ({e}); retrying every second")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self._t_retry = time.time()
            return
        for seq, d in self.parser.feed(chunk):
            if self.last_seq is not None:
                gap = (seq - self.last_seq) % 60000
                if gap != 1:
                    self.lost += gap - 1
            self.last_seq = seq
            ns = self.stamp(d)
            st = Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
            if 0x10 in d and 0x20 in d:
                m = Imu()
                m.header.stamp, m.header.frame_id = st, self.frame_id
                a, g = d[0x10], d[0x20]
                m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = a
                m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (v * DEG for v in g)
                if 0x41 in d:
                    q = d[0x41]                                    # q0 is the scalar part
                    m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = q
                else:
                    m.orientation_covariance[0] = -1.0             # "no orientation" (REP-145)
                self.pub_imu.publish(m)
                self.n += 1
            if 0x31 in d:
                mg = MagneticField()
                mg.header.stamp, mg.header.frame_id = st, self.frame_id
                mg.magnetic_field.x, mg.magnetic_field.y, mg.magnetic_field.z = (v * 1e-7 for v in d[0x31])
                self.pub_mag.publish(mg)                           # 1 mGauss = 1e-7 T
            if 0x01 in d:
                t = Temperature()
                t.header.stamp, t.header.frame_id = st, self.frame_id
                t.temperature = float(d[0x01])
                self.pub_tmp.publish(t)
        if time.time() - self.t_report >= 10.0:
            dt = time.time() - self.t_report
            self.get_logger().info(f"{self.n / dt:.0f} Hz, lost frames {self.lost}, bad checksums {self.parser.bad}"
                                   + ("" if self.ts0 is not None else ", no IMU timestamps (host time used)"))
            self.t_report, self.n = time.time(), 0


def main():
    self_test()
    rclpy.init()
    node = H30wpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.ser.close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
