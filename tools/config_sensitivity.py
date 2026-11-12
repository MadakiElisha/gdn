#!/usr/bin/env python3
"""Which filter settings matter: halve and double each one and rerun the MC (R1.10).

Every key of the filter config (config/filter/default.yaml) is run at half and at twice
its value (whole-number settings and switches get the alternatives listed in VARIANTS),
through mc_campaign --set, on the R1.8 scoring scenario: GNSS lost at 150 s on the long
Zermatt flight, node start-up, SRTM-like onboard maps. Each variant is run on the same
maps and the same flight seeds as the baseline, so a difference comes from the setting,
not from different noise draws. Pooled over the 10 GNSS-free minutes it writes, per
variant: CEP95, the share of samples over 3 sigma (REQ-INT-001), the max error, and the
flights never over 100 m.

    python3 tools/config_sensitivity.py --out docs/test_reports/test028_data/sensitivity.csv
    python3 tools/config_sensitivity.py --keys r_agl,q_baro_bias --maps 2 --per-map 10 --out /tmp/s.csv

The onboard maps are the CI's SRTM-like ensemble (data/maps/ensemble/ci_srtm_NN.bin,
made by tools/mc_map_ensemble.py --tag ci_srtm); missing ones are made with the same
recipe and seeds.
"""
import argparse
import csv
import os
import re
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
RECIPE = "--coarsen 3 --noise 6 --corr 200 --bias 3"    # SRTM-like (R1.8)
SITE_KEYS = {"mag_ref_north", "mag_ref_east", "mag_ref_down"}   # the site, not tuning
# Settings that aren't scaled: the alternatives tried instead.
VARIANTS = {"static_n": ["250", "1000"], "mag_auto_hold": ["10", "30"], "mag_fusion": ["heading", "full"]}


def read_config(path):
    vals = {}
    for line in open(path):
        line = line.split("#", 1)[0].strip()
        if ":" in line:
            k, v = (x.strip() for x in line.split(":", 1))
            if k not in ("schema", "name", "source"):
                vals[k] = v
    return vals


def variants(vals, keys):
    out = [("(default)", "", "")]
    for k in keys:
        if k in VARIANTS:
            out += [(k, v, v) for v in VARIANTS[k]]
        else:
            x = float(vals[k])
            out += [(k, "x0.5", repr(x * 0.5)), (k, "x2", repr(x * 2.0))]
    return out


def score(traces, loss, dur):
    err, over, seeds_max = [], [], {}
    for tr in traces:
        with open(tr) as f:
            for r in csv.DictReader(f):
                t = float(r["t"])
                if loss <= t <= loss + dur:
                    e = float(r["err"])
                    err.append(e)
                    over.append(e > 3.0 * float(r["sigma"]))
                    key = (tr, r["seed"])
                    seeds_max[key] = max(seeds_max.get(key, 0.0), e)
    err = np.array(err)
    return {"cep95_m": float(np.percentile(err, 95)), "over3_pct": 100.0 * float(np.mean(over)),
            "max_m": float(err.max()), "never_over_100": sum(m < 100.0 for m in seeds_max.values()),
            "flights": len(seeds_max)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="CSV to write")
    ap.add_argument("--config", default="config/filter/default.yaml")
    ap.add_argument("--keys", help="comma-separated subset of keys (default: every tuning key)")
    ap.add_argument("--maps", type=int, default=5)
    ap.add_argument("--per-map", type=int, default=20)
    ap.add_argument("--csv", default="data/flight_zermatt_long_csv")
    ap.add_argument("--world", default="data/maps/zermatt.bin")
    ap.add_argument("--gnss-loss", type=float, default=150.0)
    ap.add_argument("--duration", type=float, default=600.0, help="GNSS-free seconds scored")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    a = ap.parse_args()

    binary = ROOT / "install/gdn_fusion/lib/gdn_fusion/mc_campaign"
    if not binary.exists():
        sys.exit(f"build first: {binary} not found")
    env = {**os.environ, "GDN_ROOT": str(ROOT)}
    vals = read_config(ROOT / a.config)
    keys = a.keys.split(",") if a.keys else [k for k in vals if k not in SITE_KEYS]
    for k in keys:
        if k not in vals:
            sys.exit(f"unknown key {k}")

    maps = []
    for k in range(a.maps):
        db = ROOT / f"data/maps/ensemble/ci_srtm_{k:02d}.bin"
        if not db.exists():
            db.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run([sys.executable, str(ROOT / "tools/make_db_map.py"), "--in", a.world, "--out", str(db),
                            *RECIPE.split(), "--seed", str(1000 + k)], check=True, cwd=ROOT, env=env)
        maps.append(db)

    tmp = Path(tempfile.mkdtemp(prefix="gdn_sens_"))
    runs = variants(vals, keys)

    def one(job):
        i, k = job
        key, label, value = runs[i]
        trace = tmp / f"v{i:03d}_m{k}.csv"
        cmd = [str(binary), str(a.per_map), a.csv, a.world, f"--db-map={maps[k]}", f"--seed0={k * a.per_map}",
               f"--trace={trace}", "--model=agl", "--startup=node", f"--gnss-loss={a.gnss_loss}",
               f"--config={a.config}"] + ([f"--set={key}={value}"] if value else [])
        res = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT, env=env)
        fp = re.search(r"fingerprint (\w+)", res.stdout)
        return i, k, res.returncode == 0 and trace.exists(), fp.group(1) if fp else "", res.stdout[-400:]

    jobs = [(i, k) for i in range(len(runs)) for k in range(a.maps)]
    done = {}
    with ThreadPoolExecutor(a.jobs) as ex:
        for n, (i, k, ok, fp, tail) in enumerate(ex.map(one, jobs), 1):
            done[(i, k)] = (ok, fp, tail)
            if n % 25 == 0 or n == len(jobs):
                print(f"{n}/{len(jobs)} runs", flush=True)

    rows = []
    for i, (key, label, value) in enumerate(runs):
        oks = [done[(i, k)][0] for k in range(a.maps)]
        fps = {done[(i, k)][1] for k in range(a.maps)}
        row = {"key": key, "change": label, "value": value, "fingerprint": ",".join(sorted(fps))}
        if all(oks):
            row.update(score([tmp / f"v{i:03d}_m{k}.csv" for k in range(a.maps)], a.gnss_loss, a.duration))
            row["status"] = "ok"
        else:
            bad = next(done[(i, k)][2] for k in range(a.maps) if not done[(i, k)][0])
            row["status"] = "failed: " + " ".join(bad.split())[-160:]
        rows.append(row)
        print(f"{key:24s} {label:6s} {value:>22s}  " +
              (f"CEP95 {row['cep95_m']:6.1f} m  over 3 sigma {row['over3_pct']:5.2f}%  "
               f"never over 100 m {row['never_over_100']}/{row['flights']}" if row["status"] == "ok" else row["status"]))
    cols = ["key", "change", "value", "fingerprint", "status", "cep95_m", "over3_pct", "max_m", "never_over_100", "flights"]
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {a.out}: {len(rows)} variants x {a.maps} maps x {a.per_map} flights; scenario: GNSS lost at "
          f"{a.gnss_loss:.0f} s, {a.csv}, SRTM-like maps ({RECIPE})")


if __name__ == "__main__":
    main()
