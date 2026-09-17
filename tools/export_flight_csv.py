#!/usr/bin/env python3
# Re-export IMU + local-position CSVs from a rosbag so MC campaign == bench trajectory.
import sys, os, csv, yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1] if len(sys.argv) > 1 else '/home/madakie/gdn_workspace/data/flight001'
OUT = sys.argv[2] if len(sys.argv) > 2 else '/home/madakie/gdn_workspace/data/flight001_csv'

def detect_storage(bag):
    meta = os.path.join(bag, 'metadata.yaml')
    if os.path.exists(meta):
        with open(meta) as f:
            md = yaml.safe_load(f)
        info = md.get('rosbag2_bag_information', md) or {}
        sid = info.get('storage_identifier', '')
        if sid: return sid
    files = os.listdir(bag)
    for cand in ('mcap', 'sqlite3', 'db3'):
        if any(f.endswith('.' + cand) for f in files):
            return 'mcap' if cand == 'mcap' else 'sqlite3'
    return 'mcap'

sid = detect_storage(BAG)
print(f"bag storage: {sid}")
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id=sid),
            rosbag2_py.ConverterOptions('', ''))
tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}
imu_t = get_message(tmap['/fmu/out/sensor_combined'])
lpos_t = get_message(tmap['/fmu/out/vehicle_local_position'])

imu_rows, lpos_rows = [], []
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic == '/fmu/out/sensor_combined':
        m = deserialize_message(data, imu_t)
        imu_rows.append([m.timestamp, *m.gyro_rad, *m.accelerometer_m_s2])
    elif topic == '/fmu/out/vehicle_local_position':
        m = deserialize_message(data, lpos_t)
        lpos_rows.append([m.timestamp, m.x, m.y, m.z, m.vx, m.vy, m.vz, m.ax, m.ay, m.az])

with open(OUT + '/imu.csv', 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['t','gx','gy','gz','ax','ay','az']); w.writerows(imu_rows)
with open(OUT + '/lpos.csv', 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['t','x','y','z','vx','vy','vz','ax','ay','az']); w.writerows(lpos_rows)
print(f"exported {len(imu_rows)} imu, {len(lpos_rows)} lpos rows from {BAG}")
