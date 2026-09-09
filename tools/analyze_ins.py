import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DATA = ROOT / 'data' / 'flight001_csv'

print("Loading data...")
imu = pd.read_csv(DATA / 'imu.csv')
lpos = pd.read_csv(DATA / 'lpos.csv')

t0 = imu['t'].iloc[0]
imu['t_sec'] = (imu['t'] - t0)/1e6
lpos['t_sec'] = (lpos['t'] - t0)/1e6

# --- Static window: before first motion per EKF speed
speed = np.hypot(lpos['vx'], lpos['vy'])
t_move = float(lpos.loc[speed > 0.5, 't_sec'].iloc[0]) if (speed > 0.5).any() else 30.0
static = imu[imu['t_sec'] < t_move - 2.0]
print(f"Static window: 0..{t_move-2.0:.1f}s ({len(static)} samples)")

m = static[['ax','ay','az']].to_numpy().mean(axis=0)
gyro_bias = static[['gx','gy','gz']].to_numpy().mean(axis=0)

# Coarse leveling from measured gravity (FRD body, NED nav)
roll0  = np.arctan2(-m[1], -m[2])
pitch0 = np.arctan2( m[0], np.hypot(m[1], m[2]))

# In-motion alignment: heading from velocity vector once rolling fast enough
spd = speed.to_numpy()
i_mv = int(np.argmax(spd > 5.0)) if (spd > 5.0).any() else 0
yaw0 = float(np.arctan2(lpos['vy'].iloc[i_mv], lpos['vx'].iloc[i_mv]))
print(f"Initial yaw (in-motion align): {np.degrees(yaw0):.1f} deg")

R0 = R.from_euler('ZYX', [yaw0, pitch0, roll0])

g_nav = np.array([0, 0, 9.80665])
accel_bias = m - R0.inv().apply(-g_nav)   # static: meas = R0^T(-g) + bias
print(f"gyro bias [deg/s]: {np.degrees(gyro_bias).round(4)}")
print(f"accel bias [m/s^2]: {accel_bias.round(4)}")

G = imu[['gx','gy','gz']].to_numpy(); A = imu[['ax','ay','az']].to_numpy()
T = imu['t_sec'].to_numpy()
REF = lpos[['x','y','z']].to_numpy(float)
TL  = lpos['t_sec'].to_numpy()

def integrate(calib):
    pos = REF[0].copy(); vel = lpos[['vx','vy','vz']].iloc[0].to_numpy(float).copy()
    q  = R0 if calib else R.identity()
    gb = gyro_bias if calib else np.zeros(3)
    ab = accel_bias if calib else np.zeros(3)
    ts, ps = [], []
    for i in range(1, len(T)):
        dt = T[i] - T[i-1]
        if dt <= 0: continue
        q = q * R.from_rotvec((G[i]-gb)*dt)
        vel = vel + (q.apply(A[i]-ab) + g_nav)*dt
        pos = pos + vel*dt
        ts.append(T[i]); ps.append(pos.copy())
    return np.array(ts), np.array(ps)

t_n, p_n = integrate(False)
t_c, p_c = integrate(True)

def err_at(ts, ps, t):
    i = np.abs(ts-t).argmin(); j = np.abs(TL-t).argmin()
    return np.linalg.norm(ps[i]-REF[j])

for t in (60, 120, 300, 600):
    print(f"t={t:4d}s  naive {err_at(t_n,p_n,t):12.1f} m   calibrated {err_at(t_c,p_c,t):12.1f} m")

fig, ax = plt.subplots(2, 1, figsize=(12, 9), sharex=True)
ax[0].plot(TL, REF[:,0], 'b--', label='EKF truth (N)')
ax[0].plot(t_n, p_n[:,0], 'r',  label='naive INS')
ax[0].plot(t_c, p_c[:,0], 'g',  label='calibrated INS')
ax[0].set_ylabel('North (m)'); ax[0].legend(); ax[0].grid()
en = np.linalg.norm(p_n - np.array([np.interp(t_n, TL, REF[:,k]) for k in range(3)]).T, axis=1)
ec = np.linalg.norm(p_c - np.array([np.interp(t_c, TL, REF[:,k]) for k in range(3)]).T, axis=1)
ax[1].semilogy(t_n, en, 'r', label='naive error')
ax[1].semilogy(t_c, ec, 'g', label='calibrated error')
ax[1].set_xlabel('Time (s)'); ax[1].set_ylabel('3D error (m, log)')
ax[1].legend(); ax[1].grid(which='both')
plt.tight_layout()
plt.savefig(ROOT / 'data' / 'flight001_ins_v2.png', dpi=150)
print("\nSaved plot to data/flight001_ins_v2.png")
