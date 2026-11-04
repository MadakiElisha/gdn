#!/usr/bin/env python3
"""Write the Zermatt synthetic flight as a PX4-style rosbag2 (MCAP) so the full
ROS replay bench can run on a fresh clone without downloading flight001.

    source env.sh && tools/make_data.sh && python3 tools/make_zermatt_bag.py
    GDN_BAG=data/flight_zermatt GDN_MAP=data/maps/zermatt.bin tools/bench.sh

Topics: /fmu/out/sensor_combined (100 Hz), /fmu/out/vehicle_local_position and
/fmu/out/vehicle_attitude (10 Hz). Level flight at constant altitude; yaw from
the velocity vector. Needs ROS 2 + px4_msgs (built workspace sourced).
"""
import csv
import math
import shutil

import rosbag2_py
from rclpy.serialization import serialize_message
from px4_msgs.msg import SensorCombined, VehicleLocalPosition, VehicleAttitude
from gdn_paths import ROOT

SRC = ROOT / 'data/flight_zermatt_csv'
OUT = ROOT / 'data/flight_zermatt'


def rows(name):
    with open(SRC / name) as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def main():
    imu, lpos = rows('imu.csv'), rows('lpos.csv')
    if OUT.exists():
        shutil.rmtree(OUT)
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(OUT), storage_id='mcap'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for i, (name, typ) in enumerate([('/fmu/out/sensor_combined', 'px4_msgs/msg/SensorCombined'),
                                     ('/fmu/out/vehicle_local_position', 'px4_msgs/msg/VehicleLocalPosition'),
                                     ('/fmu/out/vehicle_attitude', 'px4_msgs/msg/VehicleAttitude')]):
        w.create_topic(rosbag2_py.TopicMetadata(id=i, name=name, type=typ, serialization_format='cdr'))

    t0_ns = 1_700_000_000 * 10**9          # arbitrary wall-clock origin for bag receive times
    events = []
    for r in imu:
        m = SensorCombined()
        m.timestamp = int(r['t'])
        m.gyro_rad = [r['gx'], r['gy'], r['gz']]
        m.accelerometer_m_s2 = [r['ax'], r['ay'], r['az']]
        events.append((m.timestamp, '/fmu/out/sensor_combined', m))
    # Heading while parked = heading at first motion (the generator's psi_t);
    # the gyro shows no rotation at takeoff, so attitude truth must not jump.
    yaw = next(math.atan2(r['vy'], r['vx']) for r in lpos if math.hypot(r['vx'], r['vy']) > 0.5)
    for r in lpos:
        ts = int(r['t'])
        if math.hypot(r['vx'], r['vy']) > 0.5:
            yaw = math.atan2(r['vy'], r['vx'])
        p = VehicleLocalPosition()
        p.timestamp = ts
        p.xy_valid = p.z_valid = p.v_xy_valid = p.v_z_valid = True
        p.x, p.y, p.z = r['x'], r['y'], 0.0
        p.vx, p.vy, p.vz = r['vx'], r['vy'], 0.0
        p.heading = yaw
        events.append((ts, '/fmu/out/vehicle_local_position', p))
        a = VehicleAttitude()
        a.timestamp = ts
        a.q = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]   # body->NED, w,x,y,z
        events.append((ts, '/fmu/out/vehicle_attitude', a))
    events.sort(key=lambda e: e[0])
    for ts, topic, msg in events:
        w.write(topic, serialize_message(msg), t0_ns + ts * 1000)
    del w
    (OUT / '.px4_msgs_version').write_text('1.16\n')
    print(f'wrote {OUT} ({len(events)} messages, {events[-1][0] / 1e6:.0f} s)')


if __name__ == '__main__':
    main()
