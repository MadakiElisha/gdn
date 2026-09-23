import numpy as np, struct, os
os.makedirs('data/flight_zermatt_csv', exist_ok=True)

with open('data/maps/zermatt.bin', 'rb') as f:
    nx, ny = struct.unpack('<ii', f.read(8))
    lat0, lon0, dlat, dlon = struct.unpack('<dddd', f.read(32))
    alt = np.frombuffer(f.read(4*nx*ny), dtype='<f4').reshape(ny, nx)

kLatC = 45.96319444444445
dlat_m = dlat * 111320.0
dlon_m = dlon * 111320.0 * np.cos(np.radians(kLatC))
dn, de = np.gradient(alt, dlat_m, dlon_m)

W = 24; best = None
for i0 in range(4, ny-W-4, 4):
    for j0 in range(4, nx-W-4, 4):
        c_n = (i0 + W/2 - 64) * dlat_m; c_e = (j0 + W/2 - 64) * dlon_m
        if abs(c_n) > 1100 or abs(c_e) > 800: continue
        gn = dn[i0:i0+W, j0:j0+W].ravel(); ge = de[i0:i0+W, j0:j0+W].ravel()
        G = np.array([[np.sum(gn*gn), np.sum(gn*ge)], [np.sum(gn*ge), np.sum(ge*ge)]])
        lam = np.linalg.eigvalsh(G)
        if best is None or lam[0] > best[0]:
            best = (lam[0], i0, j0)
_, i0, j0 = best
c_n = (i0 + W/2 - 64) * dlat_m; c_e = (j0 + W/2 - 64) * dlon_m
print(f"observable window: center=({c_n:.0f},{c_e:.0f}) m, lambda_min={best[0]:.1f}")

v = 30.0; a = 2.0; t_acc = v / a                      # 15 s accel
R = 150.0; w_turn = v / R; T_turn = np.pi / w_turn    # 180 deg turns
T_leg = 20.0; L = v * T_leg; sep = 2 * R
P0 = np.array([c_n + sep, c_e - L / 2.0])             # west end of first leg
psi_t = np.arctan2(P0[1], P0[0])
d_acc = 0.5 * a * t_acc ** 2
T_tr = max((np.linalg.norm(P0) - d_acc) / v, 1.0)
T_lead = 5.0
ph = {}
ph['A'] = (0.0, t_acc); ph['B'] = (t_acc, t_acc + T_tr)
ph['C'] = (ph['B'][1], ph['B'][1] + T_lead)
t = ph['C'][1]
ph['D'] = (t, t + T_leg); t += T_leg
ph['E'] = (t, t + T_turn); t += T_turn
ph['F'] = (t, t + T_leg); t += T_leg
ph['G'] = (t, t + T_turn); t += T_turn
ph['H'] = (t, t + T_leg); t += T_leg
T_tot = 30.0 + t + 5.0
lead_rate = (np.pi / 2 - psi_t) / T_lead

def kin(tc):
    psi = np.full_like(tc, psi_t); pd = np.zeros_like(tc); spd = np.zeros_like(tc)
    axl = np.zeros_like(tc)
    m = (tc > 0) & (tc < ph['A'][1])
    spd[m] = a * tc[m]; axl[m] = a; psi[m] = psi_t
    m = (tc >= ph['B'][0]) & (tc < ph['B'][1]); spd[m] = v; psi[m] = psi_t
    m = (tc >= ph['C'][0]) & (tc < ph['C'][1])
    spd[m] = v; psi[m] = psi_t + lead_rate * (tc[m] - ph['C'][0]); pd[m] = lead_rate
    m = (tc >= ph['D'][0]) & (tc < ph['D'][1]); spd[m] = v; psi[m] = np.pi / 2
    m = (tc >= ph['E'][0]) & (tc < ph['E'][1])
    spd[m] = v; psi[m] = np.pi / 2 + w_turn * (tc[m] - ph['E'][0]); pd[m] = w_turn
    m = (tc >= ph['F'][0]) & (tc < ph['F'][1]); spd[m] = v; psi[m] = 3 * np.pi / 2
    m = (tc >= ph['G'][0]) & (tc < ph['G'][1])
    spd[m] = v; psi[m] = 3 * np.pi / 2 + w_turn * (tc[m] - ph['G'][0]); pd[m] = w_turn
    m = tc >= ph['H'][0]; spd[m] = v; psi[m] = np.pi / 2
    return psi, pd, spd, axl

dt = 0.05
tcg = np.arange(0, t, dt)
psi_g, pd_g, spd_g, _ = kin(tcg)
x = np.cumsum(spd_g * np.cos(psi_g)) * dt; y = np.cumsum(spd_g * np.sin(psi_g)) * dt
print(f"track bbox {x.max()-x.min():.0f}x{y.max()-y.min():.0f} m, "
      f"max|n|={max(abs(x.min()),abs(x.max())):.0f}, max|e|={max(abs(y.min()),abs(y.max())):.0f}")

ti = np.arange(0, T_tot, 0.01); tl = np.arange(0, T_tot, 0.1)
def series(tt):
    tc = np.clip(tt - 30.0, 0, None)
    psi, pd, spd, axl = kin(tc)
    return psi, pd, spd, axl
psi_i, pd_i, spd_i, ax_i = series(ti)
az = np.full_like(ti, -9.81); gx = np.zeros_like(ti); gy = np.zeros_like(ti)
ay_i = spd_i * pd_i; gz_i = pd_i
np.random.seed(42)
ax_i = ax_i + 0.05 + np.random.normal(0, 0.1, ti.size)
ay_i = ay_i - 0.02 + np.random.normal(0, 0.1, ti.size)
az = az + 0.03 + np.random.normal(0, 0.1, ti.size)
gx = gx + 0.002 + np.random.normal(0, 0.001, ti.size)
gy = gy - 0.001 + np.random.normal(0, 0.001, ti.size)
gz_i = gz_i + 0.0015 + np.random.normal(0, 0.001, ti.size)

psi_l, pd_l, spd_l, _ = series(tl)
xl = np.zeros_like(tl); yl = np.zeros_like(tl)
for k in range(1, tl.size):
    xl[k] = xl[k-1] + spd_l[k] * np.cos(psi_l[k]) * 0.1
    yl[k] = yl[k-1] + spd_l[k] * np.sin(psi_l[k]) * 0.1

with open('data/flight_zermatt_csv/imu.csv', 'w') as f:
    f.write("t,ax,ay,az,gx,gy,gz\n")
    for i in range(ti.size):
        f.write(f"{ti[i]*1e6:.0f},{ax_i[i]:.6f},{ay_i[i]:.6f},{az[i]:.6f},{gx[i]:.6f},{gy[i]:.6f},{gz_i[i]:.6f}\n")
with open('data/flight_zermatt_csv/lpos.csv', 'w') as f:
    f.write("t,x,y,vx,vy\n")
    for i in range(tl.size):
        f.write(f"{tl[i]*1e6:.0f},{xl[i]:.6f},{yl[i]:.6f},{spd_l[i]*np.cos(psi_l[i]):.6f},{spd_l[i]*np.sin(psi_l[i]):.6f}\n")
print("generated: static 30s -> accel -> 30 m/s transit -> lawnmower on texture")
