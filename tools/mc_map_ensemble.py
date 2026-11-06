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
not tracked).
"""
import argparse
import csv
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
ap.add_argument("--window", default="60,300", help="scoring window, s (as mc_campaign)")
ap.add_argument("--tag", default="ens", help="name for the generated maps")
ap.add_argument("mc_args", nargs="*", help="after --: options for mc_campaign")
a = ap.parse_args()
w0, w1 = (float(v) for v in a.window.split(","))

binary = ROOT / "install/gdn_fusion/lib/gdn_fusion/mc_campaign"
if not binary.exists():
    sys.exit(f"build first: {binary} not found")
out_dir = ROOT / "data/maps/ensemble"
out_dir.mkdir(parents=True, exist_ok=True)
env = {"GDN_ROOT": str(ROOT), "PATH": "/usr/bin:/bin"}

per_seed = defaultdict(lambda: {"err": [], "over": [], "nees": [], "t": []})
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
    print(f"map {k:2d}: {info.split('vs truth: ')[-1]} | {line.split('passing')[-1].strip()}")
    with open(trace) as f:
        for r in csv.DictReader(f):
            t = float(r["t"])
            if w0 <= t <= w1:
                d = per_seed[int(r["seed"])]
                e, sg = float(r["err"]), float(r["sigma"])
                d["err"].append(e); d["over"].append(e > 3.0 * sg); d["nees"].append(float(r["nees"]))

pool = np.concatenate([np.array(d["err"]) for d in per_seed.values()])
over = np.concatenate([np.array(d["over"]) for d in per_seed.values()])
nees = np.concatenate([np.array(d["nees"]) for d in per_seed.values()])
rms = np.array([np.sqrt(np.mean(np.square(d["err"]))) for d in per_seed.values()])
n = len(rms); n_pass = int((rms < 100.0).sum())
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
