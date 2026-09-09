import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# 1. Load Data
print("Loading data...")
imu = pd.read_csv('../data/flight001_csv/imu.csv')
lpos = pd.read_csv('../data/flight001_csv/lpos.csv')

# Align timestamps to t=0
t0 = imu['t'].iloc[0]
imu['t_sec'] = (imu['t'] - t0) / 1e6
lpos['t_sec'] = (lpos['t'] - t0) / 1e6

# 2. INS State Initialization
pos = np.zeros(3)  # NED position (North, East, Down)
vel = np.zeros(3)  # NED velocity
q = R.from_euler('xyz', [0, 0, 0])  # Start with identity attitude

pos_history = []
t_history = []

print("Running Strapdown INS Integration...")
# 3. Integration Loop
for i in range(1, len(imu)):
    dt = imu['t_sec'].iloc[i] - imu['t_sec'].iloc[i-1]
    if dt <= 0: continue
    
    gyro = imu[['gx', 'gy', 'gz']].iloc[i].values
    accel = imu[['ax', 'ay', 'az']].iloc[i].values
    
    # Update attitude (Integrate gyro)
    rot_vec = gyro * dt
    dq = R.from_rotvec(rot_vec)
    q = q * dq
    
    # Rotate body-frame accel to NED navigation frame
    a_nav = q.apply(accel)
    
    # Add gravity. 
    # The IMU measures a_meas = a_kinematic - g_body.
    # Therefore, a_kinematic = a_meas + g_body.
    # In NED frame, gravity points DOWN (+Z), so g_nav = [0, 0, +9.80665]
    a_kinematic = a_nav + np.array([0, 0, 9.80665])
    
    # Integrate velocity
    vel = vel + a_kinematic * dt
    
    # Integrate position
    pos = pos + vel * dt
    
    pos_history.append(pos.copy())
    t_history.append(imu['t_sec'].iloc[i])

pos_history = np.array(pos_history)
t_history = np.array(t_history)

# 4. Calculate Final Error
final_time = t_history[-1]
final_ins_pos = pos_history[-1]

# Find closest EKF reference at the end
idx_end = np.abs(lpos['t_sec'] - final_time).argmin()
final_ekf_pos = lpos[['x', 'y', 'z']].iloc[idx_end].values

error = final_ins_pos - final_ekf_pos
print(f"\n--- Final Drift at {final_time:.1f}s ---")
print(f"North Error: {error[0]:.2f} m")
print(f"East Error:  {error[1]:.2f} m")
print(f"Down Error:  {error[2]:.2f} m")
print(f"Total 3D Error: {np.linalg.norm(error):.2f} m")

# 5. Plot and Save
plt.figure(figsize=(12, 6))
plt.plot(t_history, pos_history[:, 0], label='Our INS (North)', color='red')
plt.plot(lpos['t_sec'], lpos['x'], label='EKF Truth (North)', color='blue', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('North Position (m)')
plt.title('Pure Inertial Navigation vs EKF Ground Truth')
plt.legend()
plt.grid()
plt.tight_layout()

out_path = '../data/flight001_drift.png'
plt.savefig(out_path, dpi=150)
print(f"\nPlot saved to: {out_path}")
print("Open this file in Windows to see the drift curve.")
