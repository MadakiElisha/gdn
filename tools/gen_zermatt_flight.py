"""Synthetic fixed-wing flights over the Zermatt map (level, 30 m/s, 30 s parked first).

    python3 tools/gen_zermatt_flight.py               # data/flight_zermatt_csv, 177 s (the original)
    python3 tools/gen_zermatt_flight.py --minutes 15  # data/flight_zermatt_long_csv (R1.7)

The long flight exists so the Monte Carlo can cut GNSS mid-flight and still score 5
and 10 minutes without it (mc_campaign --gnss-loss=150 --gnss-free=300,600). It flies
a lawnmower over most of the map (lines 300 m apart, 2 km long, swept north then back
south on the lines in between), steering to waypoints with a 150 m turn radius.
"""
import argparse
from gdn_paths import ROOT  # repo root (GDN_ROOT or this clone)
import math
import numpy as np, struct, os, sys

_ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
_ap.add_argument("--minutes", type=float, default=0.0,
                 help="airborne minutes of the long lawnmower flight (0: the original 177 s flight)")
_ap.add_argument("--out", help="output directory (default data/flight_zermatt_csv or data/flight_zermatt_long_csv)")
_args = _ap.parse_args()
OUT = ROOT/'data/flight_zermatt_csv'
if _args.minutes > 0:
    OUT = ROOT/'data/flight_zermatt_long_csv'
if _args.out:
    OUT = ROOT/_args.out
os.makedirs(OUT, exist_ok=True)

with open(ROOT/'data/maps/zermatt.bin', 'rb') as f:
    nx, ny = struct.unpack('<ii', f.read(8))
    lat0, lon0, dlat, dlon = struct.unpack('<dddd', f.read(32))
    alt = np.frombuffer(f.read(4*nx*ny), dtype='<f4').reshape(ny, nx)


def long_flight(minutes):
    """Lawnmower with waypoint steering. Same conventions as the original flight:
    level, body x forward, ay = v * yaw rate, az = -9.81, 100 Hz IMU, 10 Hz lpos."""
    v, acc, R, dt = 30.0, 2.0, 150.0, 0.01
    w_max, k_psi = v / R, 0.5
    E0, E1, spacing = -1000.0, 1000.0, 300.0
    north = [-1500.0 + spacing * k for k in range(11)]          # -1500 .. 1500
    south = [1350.0 - spacing * k for k in range(10)]           # 1350 .. -1350
    wps = []
    for k, n in enumerate(north + south):
        ends = (E0, E1) if k % 2 == 0 else (E1, E0)
        wps += [(n, ends[0]), (n, ends[1])]
    T_park, T_air = 30.0, minutes * 60.0
    n_tot = int(round((T_park + T_air) / dt))
    t = np.arange(n_tot) * dt
    psi = np.zeros(n_tot); spd = np.zeros(n_tot); along = np.zeros(n_tot); rate = np.zeros(n_tot)
    x = np.zeros(n_tot); y = np.zeros(n_tot)
    # Scalar maths (math, not numpy) in the stepping loop, so the files and their
    # checksums don't depend on numpy's SIMD paths.
    wi = 0
    px, py = 0.0, 0.0
    h = math.atan2(wps[0][1], wps[0][0])                     # parked facing the first waypoint
    s = 0.0
    for i in range(n_tot):
        if i * dt >= T_park:
            a_cmd = acc if s < v else 0.0
            tn, te = wps[wi % len(wps)]
            if math.hypot(tn - px, te - py) < R:                # turn anticipation
                wi += 1
                tn, te = wps[wi % len(wps)]
            err = math.remainder(math.atan2(te - py, tn - px) - h, 2 * math.pi)
            r = min(w_max, max(-w_max, k_psi * err)) if s > 5.0 else 0.0
            along[i] = a_cmd
            rate[i] = r
        psi[i], spd[i], x[i], y[i] = h, s, px, py
        # advance (semi-implicit, as the filter integrates)
        s = min(v, s + along[i] * dt)
        h = h + rate[i] * dt
        px += s * math.cos(h) * dt
        py += s * math.sin(h) * dt
    imu = (along, spd * rate, np.full(n_tot, -9.81), np.zeros(n_tot), np.zeros(n_tot), rate)
    return t, psi, spd, x, y, imu, wi


if _args.minutes > 0:
    t, psi, spd, x, y, imu, wi = long_flight(_args.minutes)
    kLatC = lat0 + 0.5 * ny * dlat
    # Terrain under the track (bilinear is overkill here: nearest cell), for the summary.
    rows = np.clip(np.round(ny / 2 + x / (dlat * 111320.0)).astype(int), 0, ny - 1)
    cols = np.clip(np.round(nx / 2 + y / (dlon * 111320.0 * np.cos(np.radians(kLatC)))).astype(int), 0, nx - 1)
    terr = alt[rows, cols]
    print(f"long flight: {t[-1] + 0.01:.0f} s ({_args.minutes:g} min airborne), {wi} waypoints, "
          f"track N {x.min():.0f}..{x.max():.0f} m, E {y.min():.0f}..{y.max():.0f} m, "
          f"terrain under it {terr.min():.0f}..{terr.max():.0f} m")
    if abs(x).max() > 0.9 * abs(dlat * 111320.0) * ny / 2 or abs(y).max() > 0.9 * dlon * 111320.0 * np.cos(np.radians(kLatC)) * nx / 2:
        sys.exit("track leaves the inner 90% of the map")
    with open(OUT/'imu_truth.csv', 'w') as f:
        f.write("t,ax,ay,az,gx,gy,gz\n")
        for i in range(t.size):
            f.write(f"{t[i]*1e6:.0f}," + ",".join(f"{c[i]:.6f}" for c in imu) + "\n")
    with open(OUT/'lpos.csv', 'w') as f:
        f.write("t,x,y,vx,vy\n")
        for i in range(0, t.size, 10):
            f.write(f"{t[i]*1e6:.0f},{x[i]:.6f},{y[i]:.6f},{spd[i]*np.cos(psi[i]):.6f},{spd[i]*np.sin(psi[i]):.6f}\n")
    # The error-free IMU must reproduce the truth track by dead reckoning, or the filter
    # would be chasing a model error. Integrate it the way the generator steps the state.
    hh, ss, qx, qy, dev = psi[0], 0.0, 0.0, 0.0, 0.0
    for i in range(t.size - 1):
        ss = min(30.0, ss + float(f"{imu[0][i]:.6f}") * 0.01)        # as written to the CSV
        hh += float(f"{imu[5][i]:.6f}") * 0.01
        qx += ss * math.cos(hh) * 0.01
        qy += ss * math.sin(hh) * 0.01
        dev = max(dev, math.hypot(qx - x[i + 1], qy - y[i + 1]))
    print(f"dead-reckoning check: the error-free IMU, as written, reproduces the track within {dev:.2f} m")
    print(f"wrote {OUT}/imu_truth.csv and lpos.csv (no imu.csv: use the IMU model, --imu=auto)")
    sys.exit(0)

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
# Error-free IMU (R1.6): the Monte Carlo corrupts it with an IMU model
# (config/imu/). imu.csv below keeps its historical errors (bag, bench, CI).
imu_truth = (ax_i.copy(), ay_i.copy(), az.copy(), gx.copy(), gy.copy(), gz_i.copy())
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

with open(OUT/'imu.csv', 'w') as f:
    f.write("t,ax,ay,az,gx,gy,gz\n")
    for i in range(ti.size):
        f.write(f"{ti[i]*1e6:.0f},{ax_i[i]:.6f},{ay_i[i]:.6f},{az[i]:.6f},{gx[i]:.6f},{gy[i]:.6f},{gz_i[i]:.6f}\n")
with open(OUT/'imu_truth.csv', 'w') as f:
    f.write("t,ax,ay,az,gx,gy,gz\n")
    for i in range(ti.size):
        f.write(f"{ti[i]*1e6:.0f}," + ",".join(f"{c[i]:.6f}" for c in imu_truth) + "\n")
with open(OUT/'lpos.csv', 'w') as f:
    f.write("t,x,y,vx,vy\n")
    for i in range(tl.size):
        f.write(f"{tl[i]*1e6:.0f},{xl[i]:.6f},{yl[i]:.6f},{spd_l[i]*np.cos(psi_l[i]):.6f},{spd_l[i]*np.sin(psi_l[i]):.6f}\n")
print("generated: static 30s -> accel -> 30 m/s transit -> lawnmower on texture")
