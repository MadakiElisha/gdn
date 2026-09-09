#!/usr/bin/env python3
"""Export selected PX4 bag topics to CSV for offline analysis. Run with ROS env sourced."""
import csv, os, sys
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from px4_msgs.msg import SensorCombined, VehicleLocalPosition, VehicleGlobalPosition, SensorGps

bag = sys.argv[1] if len(sys.argv) > 1 else 'data/flight001'
out = sys.argv[2] if len(sys.argv) > 2 else bag.rstrip('/') + '_csv'
os.makedirs(out, exist_ok=True)

files = {
    '/fmu/out/sensor_combined': (SensorCombined, f'{out}/imu.csv',
        ['t', 'gx', 'gy', 'gz', 'ax', 'ay', 'az'],
        lambda m: [m.timestamp, *m.gyro_rad, *m.accelerometer_m_s2]),
    '/fmu/out/vehicle_local_position': (VehicleLocalPosition, f'{out}/lpos.csv',
        ['t', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az'],
        lambda m: [m.timestamp, m.x, m.y, m.z, m.vx, m.vy, m.vz, m.ax, m.ay, m.az]),
    '/fmu/out/vehicle_global_position': (VehicleGlobalPosition, f'{out}/global.csv',
        ['t', 'lat', 'lon', 'alt'],
        lambda m: [m.timestamp, m.lat, m.lon, m.alt]),
    '/fmu/out/vehicle_gps_position': (SensorGps, f'{out}/gps.csv',
        ['t', 'lat', 'lon', 'alt_msl'],
        lambda m: [m.timestamp, m.latitude_deg, m.longitude_deg, m.altitude_msl_m]),
}

writers, handles = {}, {}
for topic, (cls, path, header, _) in files.items():
    handles[topic] = open(path, 'w', newline='')
    writers[topic] = csv.writer(handles[topic])
    writers[topic].writerow(header)

reader = SequentialReader()
reader.open(StorageOptions(uri=bag, storage_id='mcap'),
            ConverterOptions('cdr', 'cdr'))

counts = {t: 0 for t in files}
t_min, t_max = None, None
max_speed, z_lo, z_hi = 0.0, 1e9, -1e9

while reader.has_next():
    topic, raw, _ = reader.read_next()
    if topic not in files:
        continue
    cls, path, header, extract = files[topic]
    msg = deserialize_message(raw, cls)
    writers[topic].writerow(extract(msg))
    counts[topic] += 1
    if topic == '/fmu/out/vehicle_local_position':
        t_min = msg.timestamp if t_min is None else t_min
        t_max = msg.timestamp
        max_speed = max(max_speed, (msg.vx**2 + msg.vy**2) ** 0.5)
        z_lo, z_hi = min(z_lo, msg.z), max(z_hi, msg.z)

for h in handles.values():
    h.close()

print(f'Duration: {(t_max - t_min) / 1e6:.1f} s' if t_min else 'Duration: n/a')
print(f'Max horizontal speed: {max_speed:.2f} m/s | z range: [{z_lo:.2f}, {z_hi:.2f}] m')
for t, c in counts.items():
    print(f'{c:8d}  {t}')
print(f'CSVs written to {out}/')
