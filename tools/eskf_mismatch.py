import numpy as np, pandas as pd, matplotlib.pyplot as plt, h5py
from scipy.spatial.transform import Rotation as R
from scipy.ndimage import gaussian_filter
from scipy.interpolate import RectBivariateSpline
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))

def skew(v): return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
def clamp(v, lim):
    n = np.linalg.norm(v)
    return v * (lim/n) if n > lim else v
def repair(P):
    try:
        np.linalg.cholesky(P)
        return P
    except np.linalg.LinAlgError:
        w, V = np.linalg.eigh(P)
        return (V * w.clip(1e-6, None)) @ V.T

HERE = Path(__file__).resolve().parent; DATA = HERE.parent/'data'/'flight001_csv'
MAP = HERE.parent/'data'/'maps'/'terrain.h5'
LAT_C, LON_C = 47.3977, 8.5456
M_LAT = 111320.0; M_LON = 111320.0*np.cos(np.radians(LAT_C))
rng = np.random.default_rng(7)

imu = pd.read_csv(DATA/'imu.csv'); lpos = pd.read_csv(DATA/'lpos.csv')
t0 = imu['t'].iloc[0]
T = (imu['t'].to_numpy()-t0)/1e6
xc = 0.5*(lpos['x'].min()+lpos['x'].max()); yc = 0.5*(lpos['y'].min()+lpos['y'].max())
REF = np.column_stack([lpos['x']-xc, lpos['y']-yc, lpos['z']]).astype(float)
TL  = (lpos['t'].to_numpy()-t0)/1e6

speed = np.hypot(lpos['vx'], lpos['vy']).to_numpy()
t_move = float(TL[speed>0.5][0]) if (speed>0.5).any() else 30.0
static = T < t_move-2.0
G = imu[['gx','gy','gz']].to_numpy(); A = imu[['ax','ay','az']].to_numpy()
m = A[static].mean(axis=0); gb = G[static].mean(axis=0)
roll0 = np.arctan2(-m[1],-m[2]); pitch0 = np.arctan2(m[0], np.hypot(m[1],m[2]))
i_mv = int(np.argmax(speed>5.0)) if (speed>5.0).any() else 0
yaw0 = float(np.arctan2(lpos['vy'].to_numpy()[i_mv], lpos['vx'].to_numpy()[i_mv]))
q = R.from_euler('ZYX',[yaw0,pitch0,roll0]); g_nav = np.array([0,0,9.80665])
ab = m - q.inv().apply(-g_nav)

# ONE consistent world: smoothed terrain = reality = database (filter validation test)
with h5py.File(MAP,'r') as f:
    lats = f['lats'][:]; lons = f['lons'][:]; alt_sm = gaussian_filter(f['alt'][:], 4.0)
spl = RectBivariateSpline(lats, lons, alt_sm, kx=1, ky=1)
def h_db(lat, lon): return float(spl(lat, lon, grid=False))
def grad_db(lat, lon, d=20.0):
    gn = (h_db(lat+d/M_LAT, lon)-h_db(lat-d/M_LAT, lon))/(2*d)
    ge = (h_db(lat, lon+d/M_LON)-h_db(lat, lon-d/M_LON))/(2*d)
    return gn, ge

mi = np.arange(0, len(TL), 50)
lat_t = LAT_C + REF[mi,0]/M_LAT; lon_t = LON_C + REF[mi,1]/M_LON
h_agl = np.clip(-REF[mi,2], 0.0, None)
from terrain_map import TerrainMap
tm = TerrainMap()
terr_t = tm.get_altitude_grid(lat_t, lon_t)                       # world == database
meas = (terr_t + h_agl + rng.normal(0,5,len(mi))) - (h_agl + rng.normal(0,2,len(mi)))
Tm = TL[mi]

Pk = np.zeros((15,15))
Pk[0:3,0:3] = np.eye(3)*10.0**2
Pk[3:6,3:6] = np.eye(3)*1.0**2
Pk[6:9,6:9] = np.eye(3)*np.radians(2.0)**2
Pk[9:12,9:12] = np.eye(3)*0.02**2
Pk[12:15,12:15] = np.eye(3)*np.radians(0.05)**2
Qc = np.zeros((15,15))
Qc[3:6,3:6] = np.eye(3)*0.02**2
Qc[6:9,6:9] = np.eye(3)*np.radians(0.02)**2
Qc[9:12,9:12] = np.eye(3)*(5e-4)**2
Qc[12:15,12:15] = np.eye(3)*np.radians(0.01)**2
Rtrn = 20.0**2

pos = REF[0].copy(); vel = lpos[['vx','vy','vz']].iloc[0].to_numpy(float).copy()
x = np.zeros(15)
Pout = np.zeros((len(T),3)); Pnc = np.zeros((len(T),3))
pos2 = pos.copy(); vel2 = vel.copy(); q2 = q
meas_idx = 0; n_upd = 0; n_rej = 0; n_skip = 0

for i in range(1, len(T)):
    dt = T[i]-T[i-1]
    if dt <= 0: continue
    a_b = A[i]-ab; w_b = G[i]-gb
    Rm = q.as_matrix()
    F = np.zeros((15,15))
    F[0:3,3:6] = np.eye(3)
    F[3:6,6:9] = -Rm @ skew(a_b)
    F[3:6,9:12] = -Rm
    F[6:9,6:9] = -skew(w_b)
    F[6:9,12:15] = -np.eye(3)
    Phi = np.eye(15) + F*dt
    Pk = repair(Phi @ Pk @ Phi.T + Qc*dt)
    q = q * R.from_rotvec(w_b*dt)
    vel = vel + (q.apply(a_b) + g_nav)*dt
    pos = pos + vel*dt
    q2 = q2 * R.from_rotvec(w_b*dt)
    vel2 = vel2 + (q2.apply(a_b) + g_nav)*dt
    pos2 = pos2 + vel2*dt
    Pout[i] = pos; Pnc[i] = pos2
    if meas_idx < len(Tm) and abs(T[i]-Tm[meas_idx]) < 0.5:
        lat_n = LAT_C + pos[0]/M_LAT; lon_n = LON_C + pos[1]/M_LON
        gn, ge = grad_db(lat_n, lon_n)
        if np.hypot(gn, ge) >= 0.02:
            H = np.zeros((1,15)); H[0,0] = gn; H[0,1] = ge
            innov = (meas[meas_idx] - h_db(lat_n, lon_n)) - (H @ x)[0]
            S = (H @ Pk @ H.T)[0,0] + Rtrn
            if S > 0 and abs(innov)/np.sqrt(S) < 3.0 and abs(innov) < 40.0:
                K = (Pk @ H.T)/S
                dx = (K * innov).ravel()
                dx[6:9]  = clamp(dx[6:9],  np.radians(0.2))
                dx[9:12] = clamp(dx[9:12], 0.005)
                dx[12:15]= clamp(dx[12:15], np.radians(0.05))
                x = x + dx
                IKH = np.eye(15) - K @ H
                Pk = repair(IKH @ Pk @ IKH.T + (K * Rtrn) @ K.T)
                pos = pos + x[0:3]; vel = vel + x[3:6]
                q = q * R.from_rotvec(x[6:9])
                ab = ab + x[9:12]; gb = gb + x[12:15]
                J = np.eye(15); J[6:9,6:9] = np.eye(3) - skew(x[6:9])
                Pk = repair(J @ Pk @ J.T)
                x = np.zeros(15)
                Pout[i] = pos
                n_upd += 1
            else:
                n_rej += 1
        else:
            n_skip += 1
        meas_idx += 1

REFi = np.column_stack([np.interp(T, TL, REF[:,k]) for k in range(3)])
e_nc = np.hypot(Pnc[:,0]-REFi[:,0], Pnc[:,1]-REFi[:,1])
e_ek = np.hypot(Pout[:,0]-REFi[:,0], Pout[:,1]-REFi[:,1])
w = (T>60)&(T<300)
print(f"accepted {n_upd}, gated {n_rej}, flat-skipped {n_skip}")
print(f"RMS horiz err 60-300s: uncorrected {np.sqrt((e_nc[w]**2).mean()):8.1f} m | ESKF {np.sqrt((e_ek[w]**2).mean()):7.1f} m")
print(f"MAX  horiz err 60-300s: uncorrected {e_nc[w].max():8.1f} m | ESKF {e_ek[w].max():7.1f} m")

fig, ax = plt.subplots(1, 2, figsize=(15, 6.5))
ax[0].plot(REF[:,1], REF[:,0], 'b--', label='truth')
ax[0].plot(Pnc[:,1], Pnc[:,0], 'r', lw=0.8, label='INS uncorrected')
ax[0].plot(Pout[:,1], Pout[:,0], 'g', lw=0.8, label='INS + ESKF/TRN')
ax[0].set_xlabel('East (m)'); ax[0].set_ylabel('North (m)'); ax[0].legend(); ax[0].grid(); ax[0].set_title('Trajectories')
ax[1].semilogy(T, e_nc, 'r', label='uncorrected')
ax[1].semilogy(T, e_ek, 'g', label='ESKF/TRN')
ax[1].set_xlabel('Time (s)'); ax[1].set_ylabel('Horiz error (m, log)'); ax[1].legend(); ax[1].grid(which='both'); ax[1].set_title('Recursive fusion bounds drift')
plt.tight_layout(); plt.savefig(HERE.parent/'data'/'flight001_eksf_mismatch.png', dpi=150)
print("Saved data/flight001_eksf_mismatch.png")
