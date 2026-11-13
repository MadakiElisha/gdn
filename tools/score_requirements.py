#!/usr/bin/env python3
"""Score Monte Carlo results against the agreed requirements (R1.8, docs/requirements.md).

The limits are read from docs/requirements.md itself, so the document stays the one
place they are set. Give it the vehicle class and one or more mc_campaign outputs:

    python3 tools/score_requirements.py --class fixed-wing mc_srtm_loss.txt mc_acq500.txt
    python3 tools/score_requirements.py --selfcheck      # CI

What it scores, from what mc_campaign prints:
  REQ-NAV-001..003  "GNSS-free 300 s / 600 s" lines (--gnss-loss): CEP95 against the limit,
                    and the max rule: at least 99% of flights never over the max limit.
                    The MC counts flights never over 100 m, so a max limit other than
                    100 m can't be scored from it yet (multirotor, quadplane: SITL, R2.7).
  REQ-NAV-010       the "vertical" line: altitude error p95.
  REQ-INT-001       "over 3 sigma" on each GNSS-free line, or on the whole run when there
                    are none (start-error runs): it holds in every scenario set.
  REQ-ACQ-001       the "acquisition" line (--start-error up to 500 m): at least 99% of
                    flights under the class's max limit 120 s after take-off. The MC
                    counts under 100 m, so again fixed-wing only for now.
Results are PASS, FAIL, or NOT SCORED (with the reason). Scored on realistic map error
(REQ-DB-002): perfect-map runs are a regression check, not a score.

A scorecard is the score of one filter configuration (R1.10): it prints the "filter:" line
the reports share, and refuses reports that ran with different fingerprints.

--phase1 (the Phase 1 report, TEST-030) also refuses any report that didn't run with
R1.1 to R1.7 all switched on, and names what was off. It reads what the MC says it ran
(the "MC campaign:" or "mc setup:" line, "imu:", "filter:", "scenario:", the maps), not
the options given.
"""
import argparse
import re
import sys
from pathlib import Path

DOC = Path(__file__).resolve().parent.parent / "docs/requirements.md"
CLASSES = {"multirotor": "REQ-NAV-001", "quadplane": "REQ-NAV-002", "fixed-wing": "REQ-NAV-003"}
FLIGHTS_SHARE = 0.99      # the max rule: at least this share of flights never over the limit
MC_COUNT_LIMIT = 100.0    # mc_campaign counts flights against 100 m


def read_limits(text):
    """Limits per requirement from the requirements document's tables."""
    lim = {}
    for m in re.finditer(r"^\| (REQ-NAV-00[1-3]) \|[^|]*\|([^|]*)\|([^|]*)\|", text, re.M):
        win = {}
        for dur, cell in ((300, m.group(2)), (600, m.group(3))):
            c = re.search(r"CEP95 at most ([\d.]+) m, max ([\d.]+) m", cell)
            if c:
                win[dur] = (float(c.group(1)), float(c.group(2)))
        if win:
            lim[m.group(1)] = win
    m = re.search(r"REQ-NAV-010 \| Altitude error at most ([\d.]+) m", text)
    if m:
        lim["REQ-NAV-010"] = float(m.group(1))
    m = re.search(r"REQ-INT-001 \|[^|]*?at most ([\d.]+)% of samples", text)
    if m:
        lim["REQ-INT-001"] = float(m.group(1))
    m = re.search(r"REQ-ACQ-001 \|[^|]*?up to ([\d.]+) m[^|]*?within ([\d.]+) s", text)
    if m:
        lim["REQ-ACQ-001"] = (float(m.group(1)), float(m.group(2)))
    return lim


def parse_mc(text):
    """The scored numbers from one mc_campaign (or mc_map_ensemble) output."""
    r = {"perfect_map": "(same map: no map error)" in text, "gf": {}, "vert_p95": None, "acq": [], "over3": None}
    m = re.search(r"^consistency: error > 3 x published sigma ([\d.]+)%", text, re.M)
    if m:
        r["over3"] = float(m.group(1))
    for m in re.finditer(r"^GNSS-free ([\d.]+) s: CEP50 [\d.]+ m\s+CEP95 ([\d.]+) m\s+max ([\d.]+) m\s+"
                         r"over 3 sigma ([\d.]+)%\s+flights never over 100 m: (\d+)/(\d+)", text, re.M):
        r["gf"][int(float(m.group(1)))] = {"cep95": float(m.group(2)), "max": float(m.group(3)),
                                           "over3": float(m.group(4)), "never": int(m.group(5)), "n": int(m.group(6))}
    m = re.search(r"^vertical: altitude error p50 [\d.]+ m\s+p95 ([\d.]+) m", text, re.M)
    if m:
        r["vert_p95"] = float(m.group(1))
    for m in re.finditer(r"^acquisition \(REQ-ACQ-001\): start error ([\d.]+) m: under 100 m at 120 s after "
                         r"take-off: (\d+)/(\d+) flights", text, re.M):
        r["acq"].append((float(m.group(1)), int(m.group(2)), int(m.group(3))))
    return r


def common_filter(texts):
    """(the filter line all reports share, None) or (None, why not). R1.10."""
    lines = [m.group(1).strip() if m else None
             for m in (re.search(r"^filter: (.+)$", t, re.M) for t in texts)]
    fps = {m.group(1) for m in (re.search(r"fingerprint (\w+)", l or "") for l in lines) if m}
    if len(fps) > 1:
        return None, ("the reports ran with different filter configurations (fingerprints "
                      + ", ".join(sorted(fps)) + "): score each configuration on its own")
    stated = [l for l in lines if l]
    if not stated:
        return "not stated (reports from before R1.10)", None
    if len(stated) < len(lines):
        return stated[0] + " (some reports don't state it)", None
    return stated[0], None


def phase1_missing(text):
    """What of R1.1 to R1.7 one MC report ran without ([] if all on). The Phase 1 report.

    Read from what the MC says it ran. The one allowed exception: a recorded flight
    (flight001) starts on the magnetometer heading only (--mag=align), because its CSVs
    have no attitude truth to simulate a 3-axis magnetometer from (TEST-018).
    """
    def line(prefix):
        m = re.search(rf"^{prefix}(.*)$", text, re.M)
        return m.group(1) if m else None

    miss = []
    setup = line("MC campaign: ") or line("mc setup: ")
    imu = line("imu: ") or ""
    filt = line("filter: ")
    scen = line("scenario: ") or ""
    recorded = imu.startswith("recorded")
    if setup is None:
        miss += ["R1.1: the report doesn't say which TRN model ran (no MC campaign/mc setup line)",
                 "R1.2: the report doesn't say which start-up ran"]
    else:
        if not re.search(r"\bmodel agl\b", setup):
            miss.append("R1.1: TRN model isn't agl (" + setup.split(",")[1].strip() + ")")
        if "startup node (IMU ground" not in setup:
            miss.append("R1.2: start-up isn't the node's with the IMU ground monitor")
        mag = re.search(r"\bmag (\w+)\)", setup)
        mag = mag.group(1) if mag else None
        if mag is not None and mag != "auto" and not (mag == "align" and recorded):
            miss.append(f"R1.4: magnetometer fusion is {mag}, not auto"
                        + (" (align is allowed on a recorded flight only)" if mag == "align" else ""))
    if "(same map: no map error)" in text or not (line("map ensemble: ") or line("maps: ")):
        miss.append("R1.3: no onboard map error (perfect map)")
    if line("consistency: error > 3 x published sigma") is None:
        miss.append("R1.5: no 3-sigma consistency line")
    if not (imu.startswith("h30wp") or recorded) or "process noise matched" in imu:
        miss.append("R1.6: IMU isn't the H30WP model or a recorded IMU (" + (imu.split(" (")[0] or "not stated") + ")")
    if "then lost" not in scen and "start error" not in scen:
        miss.append("R1.7: neither a GNSS loss nor a start error (" + (scen.split(";")[-1].strip() or "not stated") + ")")
    if filt is None or not filt.startswith("default (config/filter/default.yaml)"):
        miss.append("R1.10: not the default filter configuration (" + (filt or "not stated") + ")")
    else:
        ov = re.search(r"overrides (.*)$", filt)
        ov = [x.strip() for x in ov.group(1).split(",")] if ov else []
        extra = [x for x in ov if not x.startswith("mag_fusion=")]   # the mag mode is R1.4's, above
        if extra:
            miss.append("R1.10: Phase 1 is scored at the defaults, but this ran with " + ", ".join(extra))
    return miss


def score(limits, cls, runs):
    """List of (requirement, what, measured, limit, verdict) for one vehicle class."""
    req = CLASSES[cls]
    rows = []
    for run in runs:
        if run["perfect_map"]:
            rows += [(req, f"{k} at {d // 60} min", "perfect map", "-", "NOT SCORED")
                     for d in run["gf"] for k in ("CEP95", "max")]
            continue
        for d, g in sorted(run["gf"].items()):
            if d not in limits[req]:
                continue
            c95, mx = limits[req][d]
            rows.append((req, f"CEP95 at {d // 60} min", f"{g['cep95']:.1f} m", f"{c95:g} m",
                         "PASS" if g["cep95"] <= c95 else "FAIL"))
            if mx != MC_COUNT_LIMIT:
                rows.append((req, f"max at {d // 60} min", f"(MC counts {MC_COUNT_LIMIT:g} m)", f"{mx:g} m",
                             "NOT SCORED"))
            else:
                rows.append((req, f"max at {d // 60} min", f"{g['never']}/{g['n']} flights", f"{mx:g} m, 99%",
                             "PASS" if g["never"] >= FLIGHTS_SHARE * g["n"] else "FAIL"))
            rows.append(("REQ-INT-001", f"over 3 sigma at {d // 60} min", f"{g['over3']:.2f}%",
                         f"{limits['REQ-INT-001']:g}%", "PASS" if g["over3"] <= limits["REQ-INT-001"] else "FAIL"))
        if not run["gf"] and run["over3"] is not None:   # REQ-INT-001 holds in every scenario set
            rows.append(("REQ-INT-001", "over 3 sigma, whole run", f"{run['over3']:.2f}%",
                         f"{limits['REQ-INT-001']:g}%", "PASS" if run["over3"] <= limits["REQ-INT-001"] else "FAIL"))
        if run["vert_p95"] is not None and run["gf"]:
            rows.append(("REQ-NAV-010", "altitude p95", f"{run['vert_p95']:.1f} m", f"{limits['REQ-NAV-010']:g} m",
                         "PASS" if run["vert_p95"] <= limits["REQ-NAV-010"] else "FAIL"))
        start_max, _ = limits["REQ-ACQ-001"]
        mx = limits[req][300][1]
        for start, ok, n in run["acq"]:
            if start > start_max:
                continue
            if mx != MC_COUNT_LIMIT:
                rows.append(("REQ-ACQ-001", f"from {start:.0f} m", f"(MC counts {MC_COUNT_LIMIT:g} m)", f"{mx:g} m",
                             "NOT SCORED"))
            else:
                rows.append(("REQ-ACQ-001", f"from {start:.0f} m", f"{ok}/{n} flights", f"{mx:g} m at 120 s, 99%",
                             "PASS" if ok >= FLIGHTS_SHARE * n else "FAIL"))
    return rows


# ------------------------------------------------------------------- selfcheck ----
_DOC = """
| REQ-NAV-003 | Fixed-wing, cruise at 100 to 300 m above ground | CEP95 at most 50 m, max 100 m (99% of flights) | CEP95 at most 75 m, max 100 m (99% of flights) |
| REQ-NAV-001 | Multirotor, low | CEP95 at most 15 m, max 30 m (99% of flights) | CEP95 at most 25 m, max 50 m (99% of flights) |
| REQ-NAV-010 | Altitude error at most 5 m (95%) for the full GNSS-free window, all vehicles. |
| REQ-INT-001 | Consistency: horizontal error exceeds the reported 3-sigma in at most 1% of samples, in every scenario set. |
| REQ-ACQ-001 | From an initial horizontal error of up to 500 m, on rugged or rolling terrain, reach the REQ-NAV limits within 120 s of TRN-capable flight. |
"""


def _mc(c95_5, nv5, c95_10, nv10, o3=0.5, vp95=2.0, n=100):
    return (f"maps: world zermatt.bin, onboard srtm_like_0.bin\n"
            f"vertical: altitude error p50 0.5 m  p95 {vp95:.1f} m  max 3.0 m\n"
            f"GNSS-free 300 s: CEP50 10.0 m  CEP95 {c95_5:.1f} m  max 120.0 m  over 3 sigma {o3:.2f}%  "
            f"flights never over 100 m: {nv5}/{n}\n"
            f"GNSS-free 600 s: CEP50 12.0 m  CEP95 {c95_10:.1f} m  max 130.0 m  over 3 sigma {o3:.2f}%  "
            f"flights never over 100 m: {nv10}/{n}\n")


def _acq(start, ok, n=100):
    return (f"acquisition (REQ-ACQ-001): start error {start:.0f} m: under 100 m at 120 s after take-off: "
            f"{ok}/{n} flights (error there median 40.0 m); stayed under to the end (900 s): {ok}/{n}\n")


_IMU_H30WP = "imu: h30wp (WHEELTEC H30WP (Yesense), 2.25 h bench log) on imu_truth.csv"
_IMU_RECORDED = "imu: recorded (imu.csv as logged, plus a per-flight offset of 0.05 m/s^2 and 0.02 deg/s)"
_IMU_PLACEHOLDER = "imu: placeholder-px4-default (generic MEMS placeholder) on imu_truth.csv"
_SCEN_LOSS = "scenario: GNSS (simulated, 5 Hz) until t=150 s, then lost; the filter keeps its covariance (handover)."


def _phase1_on():
    """An MC report as the Phase 1 report runs it: R1.1 to R1.7 all on."""
    return (f"{_IMU_H30WP}\n"
            "filter: default (config/filter/default.yaml), fingerprint 9025345f59ba\n"
            f"{_SCEN_LOSS}\n"
            "map ensemble: 5 maps x 20 flights, recipe '--coarsen 3', mc: --model=agl, pooled n=1\n"
            "mc setup: model agl, startup node (IMU ground, mag auto), baro drift 0.0 m\n"
            "consistency: error > 3 x published sigma 0.50% (REQ-INT-001: at most 1%)\n"
            + _mc(45.0, 99, 70.0, 99))


def selfcheck():
    ok = True

    def check(cond, what):
        nonlocal ok
        print(f"{'PASS' if cond else 'FAIL'}  score: {what}")
        ok &= bool(cond)

    try:
        lim = read_limits(_DOC)
        check(lim["REQ-NAV-003"] == {300: (50.0, 100.0), 600: (75.0, 100.0)}, "reads fixed-wing CEP95 and max per window")
        check(lim["REQ-NAV-010"] == 5.0 and lim["REQ-INT-001"] == 1.0, "reads the vertical and integrity limits")
        check(lim["REQ-ACQ-001"] == (500.0, 120.0), "reads the acquisition start error and time")
        real = read_limits(DOC.read_text())
        check(all(k in real for k in ("REQ-NAV-001", "REQ-NAV-002", "REQ-NAV-003", "REQ-NAV-010", "REQ-INT-001",
                                      "REQ-ACQ-001")), "reads every scored limit from docs/requirements.md")

        def verdicts(*runs, cls="fixed-wing"):
            return {(r[0], r[1]): r[4] for r in score(lim, cls, [parse_mc(t) for t in runs])}

        v = verdicts(_mc(45.0, 99, 70.0, 99))
        check(v[("REQ-NAV-003", "CEP95 at 5 min")] == "PASS" and v[("REQ-NAV-003", "CEP95 at 10 min")] == "PASS",
              "CEP95 under the limits passes")
        check(v[("REQ-NAV-003", "max at 5 min")] == "PASS", "99 of 100 flights never over the max passes")
        v = verdicts(_mc(51.0, 98, 70.0, 99))
        check(v[("REQ-NAV-003", "CEP95 at 5 min")] == "FAIL", "CEP95 over the limit fails")
        check(v[("REQ-NAV-003", "max at 5 min")] == "FAIL", "98 of 100 flights fails the 99% max rule")
        v = verdicts(_mc(45.0, 99, 70.0, 99, o3=1.5, vp95=6.0))
        check(v[("REQ-INT-001", "over 3 sigma at 5 min")] == "FAIL" and v[("REQ-NAV-010", "altitude p95")] == "FAIL",
              "integrity over 1% and altitude p95 over 5 m fail")
        v = verdicts(_acq(500, 99), _acq(150, 50))
        check(v[("REQ-ACQ-001", "from 500 m")] == "PASS" and v[("REQ-ACQ-001", "from 150 m")] == "FAIL",
              "acquisition: each start error up to 500 m is scored")
        v = verdicts(_acq(500, 99) + "consistency: error > 3 x published sigma 3.86% (REQ-INT-001: at most 1%)\n")
        check(v.get(("REQ-INT-001", "over 3 sigma, whole run")) == "FAIL",
              "integrity is scored on every scenario set, start-error runs too")
        v = verdicts(_acq(800, 10))
        check(("REQ-ACQ-001", "from 800 m") not in v, "start errors over 500 m aren't scored")
        v = verdicts(_mc(10.0, 100, 20.0, 100), cls="multirotor")
        check(v[("REQ-NAV-001", "max at 5 min")] == "NOT SCORED" and v[("REQ-NAV-001", "CEP95 at 5 min")] == "PASS",
              "a max limit the MC doesn't count (30 m) is NOT SCORED, CEP95 still is")
        v = verdicts(_mc(45.0, 99, 70.0, 99).replace("srtm_like_0.bin", "zermatt.bin (same map: no map error)"))
        check(all(x == "NOT SCORED" for k, x in v.items() if k[0] == "REQ-NAV-003"),
              "a perfect-map run isn't scored (REQ-DB-002)")
        fa = "filter: default (config/filter/default.yaml), fingerprint 9025345f59ba\n"
        fb = "filter: default (config/filter/default.yaml), fingerprint 5b7ae4d53c18; overrides r_agl=50\n"
        same, err1 = common_filter([fa + _acq(500, 99), fa + _mc(45.0, 99, 70.0, 99)])
        _, err2 = common_filter([fa + _acq(500, 99), fb + _mc(45.0, 99, 70.0, 99)])
        check(same and "9025345f59ba" in same and err1 is None and err2 and "different filter" in err2,
              "reports from one filter configuration score together; two configurations are refused")
    except NotImplementedError:
        check(False, "scorer not implemented")
    try:
        on = _phase1_on()
        check(phase1_missing(on) == [], "phase 1: a report with R1.1 to R1.7 all on passes")

        def off(old, new, text=None):
            return phase1_missing((text or on).replace(old, new))

        def names(miss, item):
            return len(miss) == 1 and miss[0].startswith(item + ":")

        check(names(off("model agl", "model naive"), "R1.1"), "phase 1: the naive TRN model is refused (R1.1)")
        check(names(off("startup node (IMU ground, mag auto)", "startup truth"), "R1.2")
              and names(off("IMU ground", "GNSS-speed ground"), "R1.2"),
              "phase 1: truth start-up and the GNSS-speed ground rule are refused (R1.2)")
        check(names(off("onboard srtm_like_0.bin", "onboard zermatt.bin (same map: no map error)"), "R1.3"),
              "phase 1: a perfect onboard map is refused (R1.3)")
        check(names(off("mag auto", "mag full"), "R1.4"), "phase 1: 3-axis-only magnetometer is refused (R1.4)")
        rec = on.replace(_IMU_H30WP, _IMU_RECORDED)
        align = (rec.replace("mag auto", "mag align")
                 .replace("fingerprint 9025345f59ba", "fingerprint ec8c98aece91; overrides mag_fusion=align"))
        check(phase1_missing(align) == [] and names(off(_IMU_RECORDED, _IMU_H30WP, align), "R1.4"),
              "phase 1: heading-only start is allowed on a recorded flight only (no attitude truth for a mag)")
        check(names(off("consistency: error > 3 x", "consistency: (none)"), "R1.5"),
              "phase 1: a report without the 3-sigma check is refused (R1.5)")
        check(names(off(_IMU_H30WP, _IMU_PLACEHOLDER), "R1.6"), "phase 1: the placeholder IMU is refused (R1.6)")
        check(names(off(_SCEN_LOSS, "scenario: no GNSS; the filter starts at the true position (sigma 10 m per axis)"),
                    "R1.7"), "phase 1: a run with neither GNSS loss nor start error is refused (R1.7)")
        check(names(off("fingerprint 9025345f59ba", "fingerprint 5b7ae4d53c18; overrides r_agl=200"), "R1.10"),
              "phase 1: a tuned configuration is refused: Phase 1 is scored at the defaults")
        miss = phase1_missing("\n".join(l for l in on.splitlines() if not l.startswith("mc setup:")))
        check(miss and any("R1.1" in x for x in miss) and any("R1.2" in x for x in miss),
              "phase 1: a report that doesn't say how the MC ran is refused, not assumed")
    except NotImplementedError:
        check(False, "phase 1 check not implemented")
    print("SCORE SELFCHECK PASSED" if ok else "SCORE SELFCHECK FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("reports", nargs="*", help="mc_campaign outputs")
    ap.add_argument("--class", dest="cls", choices=sorted(CLASSES), help="vehicle class")
    ap.add_argument("--csv", help="also write the rows to this CSV (with --label)")
    ap.add_argument("--label", default="", help="scenario name for the CSV rows")
    ap.add_argument("--phase1", action="store_true",
                    help="refuse reports that didn't run with R1.1 to R1.7 all on (the Phase 1 report)")
    ap.add_argument("--selfcheck", action="store_true")
    a = ap.parse_args()
    if a.selfcheck:
        sys.exit(0 if selfcheck() else 1)
    if not a.cls or not a.reports:
        ap.error("give --class and at least one report")
    texts = [Path(p).read_text() for p in a.reports]
    filt, why = common_filter(texts)
    if why:
        sys.exit("ERROR: " + why)
    print(f"filter: {filt}")
    if a.phase1:
        bad = [(p, m) for p, t in zip(a.reports, texts) for m in phase1_missing(t)]
        if bad:
            sys.exit("ERROR: --phase1: not run with R1.1 to R1.7 all on:\n"
                     + "\n".join(f"  {p}: {m}" for p, m in bad))
        print("phase 1: R1.1 to R1.7 all on in every report")
    rows = score(read_limits(DOC.read_text()), a.cls, [parse_mc(t) for t in texts])
    for req, what, meas, lim, verdict in rows:
        print(f"{verdict:10s}  {req}  {what:24s}  {meas:>16s}   limit {lim}")
    if a.csv:
        import csv
        new = not Path(a.csv).exists()
        with open(a.csv, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["scenario", "class", "requirement", "what", "measured", "limit", "verdict"])
            w.writerows([a.label, a.cls, *r] for r in rows)
    fails = sum(r[4] == "FAIL" for r in rows)
    print(f"{len(rows)} scored lines: {sum(r[4] == 'PASS' for r in rows)} pass, {fails} fail, "
          f"{sum(r[4] == 'NOT SCORED' for r in rows)} not scored")
    sys.exit(0)


if __name__ == "__main__":
    main()
