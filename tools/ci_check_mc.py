#!/usr/bin/env python3
"""Fail (exit 1) if a Monte Carlo report is worse than the given thresholds.

    python3 tools/ci_check_mc.py mc.txt --max-cep50 20 --min-passing 17
    python3 tools/ci_check_mc.py mc.txt --max-over-3sigma 1 --max-nees-median 2.8
    python3 tools/ci_check_mc.py mc.txt --expect-imu h30wp
    python3 tools/ci_check_mc.py mc.txt --gnss-free-max-cep95 300=35 --gnss-free-max-over-3sigma 300=1
    python3 tools/ci_check_mc.py mc.txt --min-acquired 22

Reads the summary lines printed by mc_campaign. The thresholds are regression
guards around today's baseline, not the requirements in docs/requirements.md.
"""
import argparse
import re
import sys

ap = argparse.ArgumentParser()
ap.add_argument("report")
ap.add_argument("--max-cep50", type=float, help="fail if pooled CEP50 [m] is above this")
ap.add_argument("--max-cep95", type=float, help="fail if pooled CEP95 [m] is above this")
ap.add_argument("--min-passing", type=int, help="fail if fewer seeds pass (<100 m RMS)")
# R1.5 consistency (gdn/consistency.hpp). REQ-INT-001 is --max-over-3sigma 1.
ap.add_argument("--max-over-3sigma", type=float,
                help="fail if more than this %% of samples have error > 3 x published sigma")
ap.add_argument("--max-nees-median", type=float,
                help="fail if the horizontal NEES median is above this (consistent: 1.39)")
ap.add_argument("--max-nis-over9", type=float,
                help="fail if more than this %% of TRN/AGL updates have NIS > 9 (consistent: 0.27)")
# R1.6: results are labelled with the IMU model they used ("imu: NAME ..." line).
ap.add_argument("--expect-imu",
                help="fail unless the report's imu: line names this model (e.g. h30wp, placeholder-px4-default, recorded)")
# R1.7: "GNSS-free D s: ..." lines (mc_campaign --gnss-loss) and the acquisition line
# (--start-error). Each GNSS-free option takes D=VALUE and can be repeated.
ap.add_argument("--gnss-free-max-cep95", action="append", default=[], metavar="D=M",
                help="fail if CEP95 over the D s after GNSS loss is above M metres")
ap.add_argument("--gnss-free-max-over-3sigma", action="append", default=[], metavar="D=PCT",
                help="fail if more than PCT %% of samples in the D s after GNSS loss are over 3 sigma")
ap.add_argument("--gnss-free-min-never-over-100", action="append", default=[], metavar="D=N",
                help="fail if fewer than N flights stay under 100 m for the D s after GNSS loss")
ap.add_argument("--min-acquired", type=int,
                help="fail if fewer flights are under 100 m at 120 s after take-off (REQ-ACQ-001 line)")
a = ap.parse_args()

text = open(a.report).read()
cep = re.search(r"CEP50:\s*([0-9.]+) m\s+CEP95:\s*([0-9.]+) m", text)
passing = re.search(r"passing\(<100 m\):\s*(\d+)/(\d+)", text)
if not cep or not passing:
    sys.exit("could not find the MC summary lines in " + a.report)

cep50, cep95 = float(cep.group(1)), float(cep.group(2))
over3 = re.search(r"error > 3 x published sigma\s*([0-9.]+)%", text)
nees = re.search(r"NEES mean\s*[0-9.]+\s+median\s*([0-9.]+)", text)
nis9 = re.search(r"NIS > 9\s*([0-9.]+)%", text)
for want, found, what in ((a.max_over_3sigma, over3, "error > 3 x sigma"),
                          (a.max_nees_median, nees, "NEES"), (a.max_nis_over9, nis9, "NIS")):
    if want is not None and not found:
        sys.exit(f"threshold on {what} given, but no {what} line in {a.report}")
n_pass, n_all = int(passing.group(1)), int(passing.group(2))
print(f"CEP50 {cep50:.1f} m, CEP95 {cep95:.1f} m, passing {n_pass}/{n_all}")

failures = []
imu = re.search(r"^imu: (\S+)", text, re.M)
print("imu: " + (imu.group(1) if imu else "(no imu: line)"))
if a.expect_imu is not None and (not imu or imu.group(1) != a.expect_imu):
    failures.append(f"IMU model {imu.group(1) if imu else 'not stated'}, expected {a.expect_imu}")
if a.max_cep50 is not None and cep50 > a.max_cep50:
    failures.append(f"CEP50 {cep50:.1f} m > {a.max_cep50} m")
if a.max_cep95 is not None and cep95 > a.max_cep95:
    failures.append(f"CEP95 {cep95:.1f} m > {a.max_cep95} m")
if a.min_passing is not None and n_pass < a.min_passing:
    failures.append(f"passing seeds {n_pass} < {a.min_passing}")
if over3 or nees or nis9:
    print("consistency:" + (f" over 3 sigma {float(over3.group(1)):.2f}%" if over3 else "")
          + (f", NEES median {float(nees.group(1)):.2f}" if nees else "")
          + (f", NIS > 9 {float(nis9.group(1)):.1f}%" if nis9 else ""))
if a.max_over_3sigma is not None and float(over3.group(1)) > a.max_over_3sigma:
    failures.append(f"error > 3 sigma in {float(over3.group(1)):.2f}% of samples > {a.max_over_3sigma}%")
if a.max_nees_median is not None and float(nees.group(1)) > a.max_nees_median:
    failures.append(f"NEES median {float(nees.group(1)):.2f} > {a.max_nees_median}")
if a.max_nis_over9 is not None and float(nis9.group(1)) > a.max_nis_over9:
    failures.append(f"NIS > 9 in {float(nis9.group(1)):.1f}% of updates > {a.max_nis_over9}%")
# R1.7
gf = {float(m.group(1)): (float(m.group(2)), float(m.group(3)), float(m.group(4)), int(m.group(5)))
      for m in re.finditer(r"^GNSS-free ([0-9.]+) s: CEP50 [0-9.]+ m\s+CEP95 ([0-9.]+) m\s+max ([0-9.]+) m\s+"
                           r"over 3 sigma ([0-9.]+)%\s+flights never over 100 m: (\d+)/\d+", text, re.M)}
for d, (c95, mx, o3, nv) in sorted(gf.items()):
    print(f"GNSS-free {d:.0f} s: CEP95 {c95:.1f} m, max {mx:.1f} m, over 3 sigma {o3:.2f}%, never over 100 m {nv}")
for opts, idx, what, worse in ((a.gnss_free_max_cep95, 0, "CEP95", lambda v, lim: v > lim),
                               (a.gnss_free_max_over_3sigma, 2, "over 3 sigma %", lambda v, lim: v > lim),
                               (a.gnss_free_min_never_over_100, 3, "flights never over 100 m",
                                lambda v, lim: v < lim)):
    for o in opts:
        d, lim = (float(x) for x in o.split("="))
        if d not in gf:
            sys.exit(f"threshold for GNSS-free {d:g} s given, but no such line in {a.report}")
        if worse(gf[d][idx], lim):
            failures.append(f"GNSS-free {d:g} s: {what} {gf[d][idx]:g} (limit {lim:g})")
acq = re.search(r"under 100 m at [0-9.]+ s after take-off: (\d+)/(\d+)", text)
if acq:
    print(f"acquisition: {acq.group(1)}/{acq.group(2)} under 100 m at 120 s after take-off")
if a.min_acquired is not None:
    if not acq:
        sys.exit(f"--min-acquired given, but no acquisition line in {a.report}")
    if int(acq.group(1)) < a.min_acquired:
        failures.append(f"acquired {acq.group(1)} < {a.min_acquired}")
if failures:
    print("FAIL: " + "; ".join(failures))
    sys.exit(1)
print("OK")
