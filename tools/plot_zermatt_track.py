#!/usr/bin/env python3
"""Plot one Zermatt Monte Carlo seed: track over hillshaded terrain, error vs time,
and TRN innovations (accepted / rejected).

The log is mc_campaign's diagnostic output for one focus seed, e.g.
    install/gdn_fusion/lib/gdn_fusion/mc_campaign 24 data/flight_zermatt_csv \
        data/maps/zermatt.bin 0 > /tmp/viz_trace.log
    python3 tools/plot_zermatt_track.py --log /tmp/viz_trace.log
"""
import argparse
import re, struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm
from gdn_paths import ROOT

ap = argparse.ArgumentParser()
ap.add_argument("--log", default="/tmp/viz_trace.log", help="mc_campaign focus-seed output")
ap.add_argument("--map", default=str(ROOT / "data/maps/zermatt.bin"))
ap.add_argument("--out", default="zermatt_track.png")
args = ap.parse_args()
BIN, LOG, OUT = args.map, args.log, args.out
with open(BIN, "rb") as f:
    nx, ny = struct.unpack("<ii", f.read(8))
    lat0, lon0, dlat, dlon = struct.unpack("<dddd", f.read(32))
    alt = np.frombuffer(f.read(4*nx*ny), dtype="<f4").reshape(ny, nx).astype(np.float64)

dlat_m = dlat * 111320.0
dlon_m = dlon * 111320.0 * np.cos(np.radians(lat0 + 0.5*ny*dlat))
n = (np.arange(ny) - 0.5*ny) * dlat_m
e = (np.arange(nx) - 0.5*nx) * dlon_m

dz_dn, dz_de = np.gradient(alt, abs(dlat_m), abs(dlon_m))
az, el = np.radians(315.0), np.radians(45.0)
sx, sy = -dz_de, -dz_dn
shade = np.clip((np.cos(el)*np.cos(az)*(-sx) + np.cos(el)*np.sin(az)*(-sy) + np.sin(el))
                / np.sqrt(sx**2 + sy**2 + 1), 0, 1)
rgb = cm.terrain(plt.Normalize(alt.min(), alt.max())(alt))[:, :, :3]
rgb = rgb * (0.35 + 0.65*shade[:, :, None])

pat_e = re.compile(r"^t=\s*(\d+)\s+err=\s*([0-9.]+)\s+pos=\(\s*(-?[0-9.]+),\s*(-?[0-9.]+)\)\s+truth=\(\s*(-?[0-9.]+),\s*(-?[0-9.]+)\)")
pat_u = re.compile(r"^UPD k=(\d+).*?innov=\s*(-?[0-9.]+)\s+S=\s*([0-9.]+)\s+acc=(\d)\s+pb=\(\s*(-?[0-9.]+),\s*(-?[0-9.]+)\)")
et, ee, ep, etr, uin, uacc, up = [], [], [], [], [], [], []
for line in open(LOG):
    m = pat_e.match(line)
    if m:
        et.append(int(m.group(1))); ee.append(float(m.group(2)))
        ep.append((float(m.group(3)), float(m.group(4)))); etr.append((float(m.group(5)), float(m.group(6))))
        continue
    m = pat_u.match(line)
    if m:
        uin.append(float(m.group(2))); uacc.append(int(m.group(4))); up.append((float(m.group(5)), float(m.group(6))))
et, ee, ep, etr = map(np.array, (et, ee, ep, etr))
uin, uacc, up = np.array(uin), np.array(uacc), np.array(up)

fig = plt.figure(figsize=(11, 7))
gs = fig.add_gridspec(2, 2, width_ratios=[1.35, 1], wspace=0.25, hspace=0.3)
ax0 = fig.add_subplot(gs[:, 0])
ax0.imshow(rgb, extent=[e[0]-dlon_m/2, e[-1]+dlon_m/2, n[-1]-abs(dlat_m)/2, n[0]+abs(dlat_m)/2])
ax0.plot(etr[:, 1], etr[:, 0], '-', color='black', lw=1.6, label='truth')
ax0.plot(ep[:, 1], ep[:, 0], '-', color='red', lw=1.2, alpha=0.85, label='ESKF')
ok = uacc == 1
ax0.plot(up[ok, 1], up[ok, 0], '.', color='lime', ms=3, label='TRN accepted')
ax0.plot(up[~ok, 1], up[~ok, 0], 'x', color='orange', ms=3, label='TRN rejected')
ax0.plot(0, 0, 's', color='cyan', ms=7, label='pad / origin')
ax0.set_xlabel('east [m]'); ax0.set_ylabel('north [m]')
ax0.set_title('Zermatt SRTM: GPS-denied TRN flight (seed 0)')
ax0.legend(loc='lower left', fontsize=8)
ax1 = fig.add_subplot(gs[0, 1])
ax1.plot(et, ee, '-', color='red'); ax1.axhline(100, color='k', ls=':', lw=1)
ax1.axvspan(60, 300, color='gray', alpha=0.15)
ax1.set_ylabel('horiz error [m]'); ax1.set_title('error vs time (spec < 100 m)')
ax2 = fig.add_subplot(gs[1, 1])
ax2.scatter(np.arange(len(uin))[ok], uin[ok], s=4, color='lime', label='accepted')
ax2.scatter(np.arange(len(uin))[~ok], uin[~ok], s=6, color='orange', marker='x', label='rejected')
ax2.axhline(40, color='k', ls=':', lw=1); ax2.axhline(-40, color='k', ls=':', lw=1)
ax2.set_xlabel('update k'); ax2.set_ylabel('TRN innovation [m]'); ax2.legend(fontsize=8)
fig.savefig(OUT, dpi=150)
w = (et >= 60) & (et <= 300)
print(f"wrote {OUT}: {len(et)} track pts, {int(ok.sum())}/{len(uacc)} updates accepted")
print(f"window 60-300s: median err {np.median(ee[w]):.1f} m, max {ee[w].max():.1f} m")
