#!/usr/bin/env python3
"""Fail (exit 1) if a Monte Carlo report is worse than the given thresholds.

    python3 tools/ci_check_mc.py mc.txt --max-cep50 20 --min-passing 17

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
a = ap.parse_args()

text = open(a.report).read()
cep = re.search(r"CEP50:\s*([0-9.]+) m\s+CEP95:\s*([0-9.]+) m", text)
passing = re.search(r"passing\(<100 m\):\s*(\d+)/(\d+)", text)
if not cep or not passing:
    sys.exit("could not find the MC summary lines in " + a.report)

cep50, cep95 = float(cep.group(1)), float(cep.group(2))
n_pass, n_all = int(passing.group(1)), int(passing.group(2))
print(f"CEP50 {cep50:.1f} m, CEP95 {cep95:.1f} m, passing {n_pass}/{n_all}")

failures = []
if a.max_cep50 is not None and cep50 > a.max_cep50:
    failures.append(f"CEP50 {cep50:.1f} m > {a.max_cep50} m")
if a.max_cep95 is not None and cep95 > a.max_cep95:
    failures.append(f"CEP95 {cep95:.1f} m > {a.max_cep95} m")
if a.min_passing is not None and n_pass < a.min_passing:
    failures.append(f"passing seeds {n_pass} < {a.min_passing}")
if failures:
    print("FAIL: " + "; ".join(failures))
    sys.exit(1)
print("OK")
