import numpy as np, pandas as pd, matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from terrain_map import TerrainMap

HERE = Path(__file__).resolve().parent; DATA = HERE.parent/'data'/'flight001_csv'
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
R0 = R.from_euler('ZYX',[yaw0,pitch0,roll0]); g_nav = np.array([0,0,9.80665])
ab = m - R0.inv().apply(-g_nav)

tm = TerrainMap()
mi = np.arange(0, len(TL), 50)
lat_t = LAT_C + REF[mi,0]/M_LAT; lon_t = LON_C + REF[mi,1]/M_LON
h_agl = np.clip(-REF[mi,2], 0.0, None)
terr_t = tm.get_altitude_grid(lat_t, lon_t)
meas = (terr_t + h_agl + rng.normal(0,5,len(mi))) - (h_agl + rng.normal(0,2,len(mi)))
Tm = TL[mi]

def to_ll(pos): return LAT_C + pos[:,0]/M_LAT, LON_C + pos[:,1]/M_LON

WIN, STEP, RAD, FIX_EVERY, T_END = 10.0, 40.0, 240.0, 10.0, 300.0

def integrate(reset_fixes):
    pos = REF[0].copy()
    vel = lpos[['vx','vy','vz']].iloc[0].to_numpy(float).copy()
    q = R0
    meas_pos = np.zeros((len(Tm), 3)); meas_idx = 0
    P = np.zeros((len(T), 3)); fixes = []
    next_fix = 40.0
    for i in range(1, len(T)):
        dt = T[i]-T[i-1]
        if dt <= 0: continue
        q = q * R.from_rotvec((G[i]-gb)*dt)
        vel = vel + (q.apply(A[i]-ab) + g_nav)*dt
        pos = pos + vel*dt
        P[i] = pos
        if meas_idx < len(Tm) and abs(T[i]-Tm[meas_idx]) < 0.5:
            meas_pos[meas_idx] = pos; meas_idx += 1
        if reset_fixes and T[i] >= next_fix and T[i] < T_END:
            w = (Tm >= T[i]-WIN) & (Tm <= T[i])
            if w.sum() >= 12:
                lat_i, lon_i = to_ll(meas_pos[w])
                off = np.arange(-RAD, RAD+1, STEP)
                dN, dE = np.meshgrid(off, off, indexing='ij')
                dN = dN.ravel()[:,None]; dE = dE.ravel()[:,None]
                lq = lat_i[None,:] + dN/M_LAT; oq = lon_i[None,:] + dE/M_LON
                ok = (lq.min(1)>LAT_C-0.05)&(lq.max(1)<LAT_C+0.05)&(oq.min(1)>LON_C-0.05)&(oq.max(1)<LON_C+0.05)
                pred = tm.get_altitude_grid(lq, oq)
                d = meas[w][None,:] - pred
                cost = np.where(ok, np.mean(np.abs(d - np.median(d,axis=1,keepdims=True)),axis=1), np.inf)
                k = int(np.argmin(cost)); s = np.sort(cost)
                ratio = s[1]/max(s[0],1e-9)
                if ratio > 1.25 and s[0] < 12.0:
                    fix = P[i].copy()
                    fix[0] += off[k//len(off)]; fix[1] += off[k%len(off)]
                    err = np.hypot(fix[0]-REF[i,0], fix[1]-REF[i,1])
                    fixes.append((T[i], err, ratio))
                    print(f"FIX    t={T[i]:6.1f}s err={err:7.1f} m ratio={ratio:5.2f} cost={s[0]:5.1f}")
                    pos = fix; P[i] = pos
                else:
                    print(f"REJECT t={T[i]:6.1f}s ratio={ratio:5.2f} cost={s[0]:5.1f}")
            next_fix += FIX_EVERY
    return P, fixes

P_nc, _   = integrate(False)
P_fx, fixes = integrate(True)

REFi = np.column_stack([np.interp(T, TL, REF[:,k]) for k in range(3)])
fig, ax = plt.subplots(1, 2, figsize=(15, 6.5))
ax[0].plot(REF[:,1], REF[:,0], 'b--', label='truth')
ax[0].plot(P_nc[:,1], P_nc[:,0], 'r', lw=0.8, label='INS uncorrected')
ax[0].plot(P_fx[:,1], P_fx[:,0], 'g', lw=0.8, label='INS + TRN fixes')
ax[0].set_xlabel('East (m)'); ax[0].set_ylabel('North (m)'); ax[0].legend(); ax[0].grid(); ax[0].set_title('Trajectories')
e_nc = np.hypot(P_nc[:,0]-REFi[:,0], P_nc[:,1]-REFi[:,1])
e_fx = np.hypot(P_fx[:,0]-REFi[:,0], P_fx[:,1]-REFi[:,1])
ax[1].semilogy(T, e_nc, 'r', label='uncorrected')
ax[1].semilogy(T, e_fx, 'g', label='TRN-corrected')
for tf, ef, _ in fixes: ax[1].axvline(tf, color='k', lw=0.3, alpha=0.4)
ax[1].set_xlabel('Time (s)'); ax[1].set_ylabel('Horiz error (m, log)'); ax[1].legend(); ax[1].grid(which='both'); ax[1].set_title('Gated TRN resets bound the drift')
plt.tight_layout(); plt.savefig(HERE.parent/'data'/'flight001_trn.png', dpi=150)
print("Saved data/flight001_trn.png")
