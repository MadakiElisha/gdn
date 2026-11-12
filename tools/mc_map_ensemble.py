#!/usr/bin/env python3
"""Monte Carlo over several onboard-map error patterns, pooled (R1.3, TEST-020).

A degraded onboard map is one fixed draw of map error, shared by every flight in an
MC run, so one map can be lucky or unlucky (TEST-020: 5 m of noise scored worse than
10 m on single maps). This builds --maps maps from one recipe with different noise
seeds, flies --per-map flights over each (distinct flight seeds), and pools every
sample. Output uses mc_campaign's summary format, so ci_check_mc.py reads it.

    python3 tools/mc_map_ensemble.py --world data/maps/zermatt.bin \\
        --csv data/flight_zermatt_csv --recipe "--noise 5 --corr 150" \\
        --maps 10 --per-map 20 -- --model=agl --startup=node

Everything after "--" goes to mc_campaign. Maps go to data/maps/ensemble/ (generated,
not tracked). With --gnss-loss=T among them (R1.7), the pooled window starts at the loss
and the "GNSS-free D s" lines are pooled too (--gnss-free=..., default 300,600).
"""
import argparse
import csv
import re
import shlex
import subprocess
import sys
from collections import defaultdict

import numpy as np

from gdn_paths import ROOT

ap = argparse.ArgumentParser()
ap.add_argument("--world", required=True, help="truth map")
ap.add_argument("--csv", required=True, help="flight CSV directory")
ap.add_argument("--recipe", required=True, help='make_db_map.py options, e.g. "--noise 5 --corr 150"')
ap.add_argument("--maps", type=int, default=10, help="number of map error patterns")
ap.add_argument("--per-map", type=int, default=20, help="flights per map")
ap.add_argument("--window", help="scoring window, s (as mc_campaign; default 60,300, or from the GNSS loss)")
ap.add_argument("--tag", default="ens", help="name for the generated maps")
ap.add_argument("mc_args", nargs="*", help="after --: options for mc_campaign")
a = ap.parse_args()
# R1.7: GNSS loss and GNSS-free durations, as mc_campaign reads them.
gnss_loss = next((float(x.split("=", 1)[1]) for x in a.mc_args if x.startswith("--gnss-loss=")), None)
gnss_free = next(([float(v) for v in x.split("=", 1)[1].split(",") if v]
                  for x in a.mc_args if x.startswith("--gnss-free=")), [300.0, 600.0])
gnss_free = sorted(gnss_free) if gnss_loss is not None else []
if a.window is None:
    a.window = f"{gnss_loss:g},{gnss_loss + gnss_free[-1]:g}" if gnss_loss is not None else "60,300"
w0, w1 = (float(v) for v in a.window.split(","))

binary = ROOT / "install/gdn_fusion/lib/gdn_fusion/mc_campaign"
if not binary.exists():
    sys.exit(f"build first: {binary} not found")
out_dir = ROOT / "data/maps/ensemble"
out_dir.mkdir(parents=True, exist_ok=True)
env = {"GDN_ROOT": str(ROOT), "PATH": "/usr/bin:/bin"}

imu_line = scen_line = filter_line = ""
acq = defaultdict(lambda: [0, 0])     # start error -> [under 100 m at 120 s, flights], summed over maps
vert = []                             # per map: altitude error p50, p95, max
per_seed = defaultdict(lambda: {"err": [], "over": [], "nees": [], "t": []})
gf = [defaultdict(lambda: {"err": [], "over": []}) for _ in gnss_free]   # R1.7
for k in range(a.maps):
    db = out_dir / f"{a.tag}_{k:02d}.bin"
    mk = [sys.executable, str(ROOT / "tools/make_db_map.py"), "--in", a.world, "--out", str(db),
          *shlex.split(a.recipe), "--seed", str(1000 + k)]
    info = subprocess.run(mk, check=True, capture_output=True, text=True, cwd=ROOT, env=env).stdout.strip()
    trace = out_dir / f"{a.tag}_{k:02d}.csv"
    mc = [str(binary), str(a.per_map), a.csv, a.world, f"--db-map={db}", f"--seed0={k * a.per_map}",
          f"--trace={trace}", f"--window={a.window}", *a.mc_args]
    res = subprocess.run(mc, check=True, capture_output=True, text=True, cwd=ROOT, env=env).stdout
    line = next((l for l in res.splitlines() if l.startswith("per-seed RMS")), "")
    imu_line = next((l for l in res.splitlines() if l.startswith("imu:")), imu_line if k else "")
    scen_line = next((l for l in res.splitlines() if l.startswith("scenario:")), scen_line)
    fl = next((l for l in res.splitlines() if l.startswith("filter:")), "")
    if k and fl != filter_line:   # R1.10: the same args give the same config; check anyway
        sys.exit(f"map {k}: filter configuration changed between runs:\n  {filter_line}\n  {fl}")
    filter_line = fl
    print(f"map {k:2d}: {info.split('vs truth: ')[-1]} | {line.split('passing')[-1].strip()}")
    for m in re.finditer(r"^acquisition \(REQ-ACQ-001\): start error ([0-9.]+) m: .*?: (\d+)/(\d+) flights", res, re.M):
        acq[m.group(1)][0] += int(m.group(2)); acq[m.group(1)][1] += int(m.group(3))
    m = re.search(r"^vertical: altitude error p50 ([0-9.]+) m\s+p95 ([0-9.]+) m\s+max ([0-9.]+) m", res, re.M)
    if m:
        vert.append([float(x) for x in m.groups()])
    with open(trace) as f:
        for r in csv.DictReader(f):
            t = float(r["t"])
            e, sg = float(r["err"]), float(r["sigma"])
            if w0 <= t <= w1:
                d = per_seed[int(r["seed"])]
                d["err"].append(e); d["over"].append(e > 3.0 * sg); d["nees"].append(float(r["nees"]))
            for i, dur in enumerate(gnss_free):
                if gnss_loss <= t <= gnss_loss + dur:
                    g = gf[i][int(r["seed"])]
                    g["err"].append(e); g["over"].append(e > 3.0 * sg)

pool = np.concatenate([np.array(d["err"]) for d in per_seed.values()])
over = np.concatenate([np.array(d["over"]) for d in per_seed.values()])
nees = np.concatenate([np.array(d["nees"]) for d in per_seed.values()])
rms = np.array([np.sqrt(np.mean(np.square(d["err"]))) for d in per_seed.values()])
n = len(rms); n_pass = int((rms < 100.0).sum())
print(imu_line)   # R1.6: which IMU model the results are for
if filter_line:
    print(filter_line)   # R1.10: which filter configuration, with its fingerprint
if scen_line:
    print(scen_line)   # R1.7: GNSS loss / start error
print(f"map ensemble: {a.maps} maps x {a.per_map} flights, recipe '{a.recipe}', "
      f"mc: {' '.join(a.mc_args)}, pooled n={len(pool)}")
print(f"CEP50: {np.percentile(pool, 50):.1f} m  CEP95: {np.percentile(pool, 95):.1f} m  "
      f"MAX: {pool.max():.1f} m  in-spec(<100 m): {100 * np.mean(pool < 100):.1f}%")
print(f"per-seed RMS: median {np.median(rms):.1f} m  p95 {np.percentile(rms, 95):.1f} m  "
      f"worst {rms.max():.1f} m  passing(<100 m): {n_pass}/{n}")
print(f"consistency: NEES mean {nees.mean():.2f} median {np.median(nees):.2f} (consistent: 2, 1.39), "
      f"outside 3-sigma ellipse {100 * np.mean(nees > 9):.1f}% (1.1%)")
print(f"consistency: error > 3 x published sigma {100 * over.mean():.2f}% (REQ-INT-001: at most 1%)")
# NIS isn't in the trace, so there is no pooled NIS line (ci_check_mc.py then refuses
# --max-nis-over9 on this output rather than passing it silently).
for i, dur in enumerate(gnss_free):
    e = np.concatenate([np.array(g["err"]) for g in gf[i].values()])
    o = np.concatenate([np.array(g["over"]) for g in gf[i].values()])
    never = sum(1 for g in gf[i].values() if max(g["err"]) < 100.0)
    print(f"GNSS-free {dur:.0f} s: CEP50 {np.percentile(e, 50):.1f} m  CEP95 {np.percentile(e, 95):.1f} m  "
          f"max {e.max():.1f} m  over 3 sigma {100 * o.mean():.2f}%  flights never over 100 m: {never}/{len(gf[i])}")
# R1.8: acquisition counts add up exactly over the maps. The altitude error isn't in the
# trace, so the vertical line is the largest of the per-map values: on the safe side for
# REQ-NAV-010 (tools/score_requirements.py reads both lines).
if vert:
    v = np.max(np.array(vert), axis=0)
    print(f"vertical: altitude error p50 {v[0]:.1f} m  p95 {v[1]:.1f} m  max {v[2]:.1f} m (largest of the {len(vert)} maps)")
for start, (ok, nn) in acq.items():
    print(f"acquisition (REQ-ACQ-001): start error {float(start):.0f} m: under 100 m at 120 s after take-off: "
          f"{ok}/{nn} flights (pooled over {a.maps} maps)")
