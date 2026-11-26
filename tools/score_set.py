#!/usr/bin/env python3
"""Score a set of SITL runs per scenario, and compare it with the accepted baseline
(roadmap R2.7b, TEST-043).

    python3 tools/score_set.py RUN_DIR [RUN_DIR ...]            # pool, compare, write set.md/.json/.png
    python3 tools/score_set.py log/r27b --out log/r27b/set      # a directory: every run under it
    python3 tools/score_set.py RUN_DIR ... --accept             # also make this set the baseline
    python3 tools/score_set.py --selfcheck                      # pooling and comparing on made-up data (CI)

RUN_DIR is a tools/run_scenario.py run that tools/score_run.py has scored (metrics.json,
series.npz); a directory without metrics.json is searched for runs. Needs numpy and
matplotlib, not ROS.

Pooled per scenario, as the requirements count (docs/requirements.md):
  - REQ-NAV-001 to 003: CEP95 over every sample of the set's flights in the window, and the
    max rule over flights (the share of flights never over the max limit: 99% needed);
  - REQ-NAV-010: altitude error p95 over every sample;
  - REQ-INT-001: the share of samples over 3 sigma, "in every scenario set";
  - REQ-INT-002 and 003 (the proxy until R4.4, tools/score_run.py): the share of lost-lock
    cases flagged within 30 s, and false alarms per 10 hours over the nominal flights
    (no sensor faults, no start error); these also pool over every scenario;
  - REQ-ACQ-001: the share of flights under the max limit at 120 s, from a start error;
  - cost: eskf_node's CPU, memory and processing latency (worst run).
A few runs can't show a 99% rule (that needs a hundred flights and more: R2.8, R5.5);
the report says how many flights each figure stands on.

The baseline (agreed): config/baselines/sitl.json holds, per scenario, each accepted
run's metrics (several runs: SITL runs differ a lot, TEST-034). A new set is compared per
metric: its median against the baseline's runs.
  - Gated metrics (METRICS' last field) are WORSE when past the baseline's worst run by
    more than the metric's tolerance, BETTER when past its best, else within. Any gated
    WORSE: exit code 1. The tolerances sit above the run-to-run spread TEST-043 measured
    (leave one out, 30 runs): eskf_node's memory 1.06 x the worst other run, its CPU and
    processing time 1.5 x (they move with the machine's state between sessions), the
    error at 120 s from a start error 1.17 x.
  - The navigation, consistency and integrity metrics are reported, not gated: where the
    set sits against the baseline's runs (above all of them, below all, or among them)
    and how many times the worst. With GDN's drift unbounded (OI-012), one scenario's
    runs span up to 33 x (TEST-043): a few runs can't tell a regression from that. The
    report's spread table (leave one out) shows how far an unchanged GDN lands.
Changing the baseline is explicit: --accept
(every run must have passed its scenario check; --min-runs per scenario, 3 by default)
writes the set's scenarios into it, keeping the others; commit it in a PR, with the
report that justifies it. The baseline also keeps each scenario file's hash: a changed
scenario is reported, as its runs aren't comparable.
"""
import argparse
import datetime
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "config" / "baselines" / "sitl.json"
WINDOWS = ("300", "600")
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#d9d8d4"
BLUE, ORANGE, GREEN, RED = "#2a78d6", "#eb6834", "#1f9e6a", "#b3261e"


def _get(m, path):
    for k in path.split("."):
        if not isinstance(m, dict) or k not in m:
            return float("nan")
        m = m[k]
    return float(m) if isinstance(m, (int, float)) and m is not None else float("nan")


def _nees_dev(m):
    v = _get(m, "consistency.nees_mean")
    return abs(math.log10(v / 2.0)) if v > 0 and math.isfinite(v) else float("nan")


def _acq(m):
    return _get(m, "acquisition.error_m") if m.get("acquisition", {}).get("judged") else float("nan")


# The compared metrics: key, label, unit, value from metrics.json, which way is better,
# relative and absolute tolerance (past the baseline's worst run by more than
# max(abs, rel x worst) is WORSE), and whether it's gated (WORSE fails the comparison) or
# only reported. Tolerances: above TEST-043's leave-one-out spread (in the docstring).
METRICS = [
    ("cep95_5min", "CEP95, 5 min", "m", lambda m: _get(m, "windows.300.cep95_m"), "lower", 0.0, 0.0, False),
    ("max_5min", "max, 5 min", "m", lambda m: _get(m, "windows.300.max_m"), "lower", 0.0, 0.0, False),
    ("cep95_10min", "CEP95, 10 min", "m", lambda m: _get(m, "windows.600.cep95_m"), "lower", 0.0, 0.0, False),
    ("max_10min", "max, 10 min", "m", lambda m: _get(m, "windows.600.max_m"), "lower", 0.0, 0.0, False),
    ("alt_p95", "altitude p95", "m", lambda m: _get(m, "vertical.p95_m"), "lower", 0.0, 0.0, False),
    ("over3sigma", "over 3 sigma", "%", lambda m: _get(m, "consistency.over3sigma_pct"), "lower", 0.0, 0.0, False),
    ("nees_dev", "NEES mean, decades from 2", "", _nees_dev, "lower", 0.0, 0.0, False),
    ("int_detect", "lost lock flagged (proxy)", "%", lambda m: _get(m, "integrity.detection_pct"), "higher", 0.0, 0.0,
     False),
    ("int_false", "false alarms (proxy)", "", lambda m: _get(m, "integrity.false_alarms"), "lower", 0.0, 0.0, False),
    ("acq_120s", "error at 120 s (start error)", "m", _acq, "lower", 0.50, 50.0, True),
    ("cpu_mean", "eskf_node CPU mean", "% core", lambda m: _get(m, "cpu.mean_pct"), "lower", 0.75, 1.0, True),
    ("rss_max", "eskf_node memory max", "MB", lambda m: _get(m, "cpu.rss_max_mb"), "lower", 0.20, 5.0, True),
    ("proc_p95", "processing p95 per IMU sample", "ms", lambda m: _get(m, "latency_ms.processing_p95"), "lower",
     0.75, 0.02, True),
]


def values(m):
    return {k: fn(m) for k, _, _, fn, *_ in METRICS}


# --- loading ---
def find_runs(paths):
    out = []
    for p in map(Path, paths):
        if (p / "metrics.json").exists():
            out.append(p)
        elif p.is_dir():
            out += sorted(q.parent for q in p.rglob("metrics.json") if "sitl_bag" not in q.parts)
    seen, uniq = set(), []
    for p in out:
        if p.resolve() not in seen:
            seen.add(p.resolve())
            uniq.append(p)
    return uniq


def _rel(p):
    p = Path(p).resolve()
    try:
        return str(p.relative_to(ROOT))
    except ValueError:
        return str(p)


def load(paths):
    runs = []
    for p in find_runs(paths):
        m = json.loads((p / "metrics.json").read_text())
        res = json.loads((p / "result.json").read_text()) if (p / "result.json").exists() else {}
        sp = p / "series.npz"
        series = dict(np.load(sp)) if sp.exists() else None
        sc = (p / "scenario.yaml").read_bytes() if (p / "scenario.yaml").exists() else b""
        runs.append({"dir": _rel(p), "metrics": m, "series": series, "passed": bool(res.get("passed", False)),
                     "version": res.get("gdn_version", "?"), "started": res.get("started_utc", "?"),
                     "scenario_sha": hashlib.sha256(sc).hexdigest()[:16]})
    return runs


# --- pooling ---
def pool_scenario(runs):
    ms = [r["metrics"] for r in runs]
    m0 = ms[0]
    out = {"scenario": m0["scenario"], "vehicle": m0["vehicle"], "requirement": m0["requirement"],
           "map_condition": m0["map_condition"], "runs": len(runs), "passed_check": sum(r["passed"] for r in runs),
           "flight_min": float(sum(m["route_s"] for m in ms) / 60.0), "windows": {}}
    for w in WINDOWS:
        ws = [m["windows"][w] for m in ms]
        errs = []
        for r in runs:
            s = r["series"]
            if s is not None:
                sel = s["fly"].astype(bool) & (s["t"] >= 0) & (s["t"] <= float(w))
                errs.append(s["err"][sel])
        e = np.concatenate(errs) if errs else np.zeros(0)
        out["windows"][w] = {
            "flights": len(ws), "complete": int(sum(x["complete"] for x in ws)), "samples": int(len(e)),
            "cep95_m": float(np.percentile(e, 95)) if len(e) else float("nan"),
            "cep95_limit_m": ws[0]["cep95_limit_m"], "max_limit_m": ws[0]["max_limit_m"],
            "max_median_m": float(np.median([x["max_m"] for x in ws])),
            "flights_under_max": int(sum(x["max_ok"] for x in ws)),
            "flights_under_max_pct": float(100.0 * np.mean([x["max_ok"] for x in ws]))}
        x = out["windows"][w]
        x["cep95_ok"] = bool(x["cep95_m"] <= x["cep95_limit_m"])
        x["max_rule_ok"] = bool(x["flights_under_max_pct"] >= 99.0)
    ser = [r["series"] for r in runs if r["series"] is not None]
    if ser:
        fly = [s["fly"].astype(bool) for s in ser]
        over = np.concatenate([(s["err"] > 3 * s["sig"])[f] for s, f in zip(ser, fly)])
        nees = np.concatenate([s["nees"][f] for s, f in zip(ser, fly)])
        ve = np.concatenate([s["verr"][f] for s, f in zip(ser, fly)])
        ve = ve[np.isfinite(ve)]
        out["over3sigma_pct"] = float(100 * over.mean()) if len(over) else float("nan")
        out["nees_mean"] = float(np.mean(np.minimum(nees, 1e6))) if len(nees) else float("nan")
        out["alt_p95_m"] = float(np.percentile(ve, 95)) if len(ve) else float("nan")
    out["over3sigma_limit_pct"] = m0["consistency"]["limit_pct"]
    out["alt_limit_m"] = m0.get("vertical", {}).get("limit_m", float("nan"))
    ig = [m["integrity"] for m in ms if "integrity" in m]
    if ig:
        out["integrity"] = {"judged": sum(x["judged"] for x in ig), "detected": sum(x["detected"] for x in ig),
                            "nominal_h": sum(x["flight_h"] for x in ig if x["nominal"]),
                            "false_alarms": sum(x["false_alarms"] for x in ig if x["nominal"]),
                            "flag_pct_median": float(np.median([x["flag_pct"] for x in ig])),
                            "req_002_pct": ig[0]["req_int_002_pct"], "req_003_per_10h": ig[0]["req_int_003_per_10h"]}
    acq = [m["acquisition"] for m in ms if m["acquisition"].get("judged")]
    if acq:
        out["acquisition"] = {"flights": len(acq), "ok": int(sum(a["ok"] for a in acq)),
                              "error_median_m": float(np.median([a["error_m"] for a in acq])),
                              "limit_m": acq[0]["limit_m"], "at_s": acq[0]["error_at_s"]}
    cp = [m["cpu"] for m in ms if "cpu" in m]
    la = [m["latency_ms"] for m in ms if "latency_ms" in m]
    rtf = [m["sim"]["rtf_min_30s"] for m in ms if "sim" in m]
    out["cost"] = {"cpu_mean_max_pct": max((c["mean_pct"] for c in cp), default=float("nan")),
                   "rss_max_mb": max((c["rss_max_mb"] for c in cp), default=float("nan")),
                   "proc_p95_max_ms": max((x["processing_p95"] for x in la), default=float("nan")),
                   "rtf_min": min(rtf, default=float("nan"))}
    return out


def pool(runs):
    by = {}
    for r in runs:
        by.setdefault(r["metrics"]["scenario"], []).append(r)
    sc = {k: pool_scenario(v) for k, v in sorted(by.items())}
    ig = [s["integrity"] for s in sc.values() if "integrity" in s]
    tot = {"runs": len(runs), "scenarios": len(sc)}
    if ig:
        tot.update({"judged": sum(x["judged"] for x in ig), "detected": sum(x["detected"] for x in ig),
                    "nominal_h": sum(x["nominal_h"] for x in ig), "false_alarms": sum(x["false_alarms"] for x in ig)})
        tot["detection_pct"] = 100.0 * tot["detected"] / tot["judged"] if tot["judged"] else float("nan")
        tot["false_per_10h"] = 10.0 * tot["false_alarms"] / tot["nominal_h"] if tot["nominal_h"] else float("nan")
    return {"scenarios": sc, "total": tot, "by_run": by}


def summary(p):
    lines = []
    for name, s in p["scenarios"].items():
        w = s["windows"]["300"]
        lines.append(f"{name}: {s['runs']} runs; 5 min: pooled CEP95 {w['cep95_m']:.1f} m (limit "
                     f"{w['cep95_limit_m']:g}), flights never over {w['max_limit_m']:g} m: {w['flights_under_max']} of "
                     f"{w['flights']} (99% needed); over 3 sigma {s.get('over3sigma_pct', float('nan')):.1f}%")
    t = p["total"]
    if "judged" in t:
        lines.append(f"integrity (proxy): lost lock flagged within 30 s in {t['detected']} of {t['judged']} cases; "
                     f"{t['false_alarms']} false alarms in {t['nominal_h']:.2f} h of nominal flight")
    return "\n".join(lines)


# --- the baseline ---
def read_baseline(path=BASELINE):
    path = Path(path)
    return json.loads(path.read_text()) if path.exists() else {"schema": 1, "scenarios": {}}


def judge(base_vals, new_vals, better, rel, abs_):
    """WORSE / BETTER / within, for one metric: the new set's median against the
    baseline's runs (worst and best), with the tolerance max(abs, rel x |worst|)."""
    b = [v for v in base_vals if v is not None and math.isfinite(v)]
    n = [v for v in new_vals if v is not None and math.isfinite(v)]
    if not b or not n:
        return "n/a", float("nan")
    med = float(np.median(n))
    if better == "lower":
        worst, best = max(b), min(b)
        tol = max(abs_, rel * abs(worst))
        return ("WORSE" if med > worst + tol else "BETTER" if med < best - max(abs_, rel * abs(best)) else "within"), med
    worst, best = min(b), max(b)
    tol = max(abs_, rel * abs(worst))
    return ("WORSE" if med < worst - tol else "BETTER" if med > best + max(abs_, rel * abs(best)) else "within"), med


def compare(p, base):
    rows, notes = [], []
    for name, runs in p["by_run"].items():
        b = base["scenarios"].get(name)
        if not b:
            notes.append(f"{name}: not in the baseline")
            continue
        shas = {r["scenario_sha"] for r in runs}
        if shas != {b["scenario_sha"]}:
            notes.append(f"{name}: the scenario file changed since the baseline was accepted: not comparable")
            continue
        new = [values(r["metrics"]) for r in runs]
        for k, label, unit, _, better, rel, abs_, gated in METRICS:
            bv = [r["values"].get(k) for r in b["runs"]]
            verdict, med = judge(bv, [v[k] for v in new], better, rel, abs_)
            if verdict == "n/a":
                continue
            if not gated:
                verdict = {"WORSE": "above all", "BETTER": "below all", "within": "among"}[verdict]
                if better == "higher":
                    verdict = {"above all": "below all", "below all": "above all"}.get(verdict, verdict)
            bf = [v for v in bv if v is not None and math.isfinite(v)]
            worst = max(bf) if better == "lower" else min(bf)
            rows.append({"scenario": name, "metric": k, "label": label, "unit": unit, "better": better,
                         "gated": gated, "base_min": min(bf), "base_max": max(bf), "base_runs": len(bf),
                         "new_median": med, "new_runs": len(new), "verdict": verdict,
                         "x_worst": med / worst if worst else float("nan")})
    return rows, notes


def loo_spread(by_run):
    """Run-to-run spread per metric, leaving one run out: each run against the worst of
    the scenario's other runs (new / worst for lower-is-better), over every scenario with
    two runs or more. How far an unchanged GDN lands from a baseline of the others."""
    out = {}
    for k, label, unit, _, better, rel, abs_, gated in METRICS:
        r = []
        for runs in by_run.values():
            vals = [values(x["metrics"])[k] for x in runs]
            for i, n in enumerate(vals):
                b = [v for j, v in enumerate(vals) if j != i and math.isfinite(v)]
                if not b or not math.isfinite(n):
                    continue
                worst = max(b) if better == "lower" else min(b)
                if better == "lower" and worst > 0:
                    r.append(n / worst)
                elif better == "higher" and n > 0:
                    r.append(worst / n)
        if r:
            out[k] = {"label": label, "n": len(r), "max": float(max(r)), "median": float(np.median(r)),
                      "over_1": int(sum(x > 1 for x in r)), "gated": gated,
                      "tolerance_x": 1 + rel if gated else None}
    return out


def git_commit():
    try:
        sha = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True,
                             text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain", "--", "src", "tools", "config"],
                               capture_output=True, text=True).stdout.strip()
        return sha + ("-dirty" if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return "?"


def accept(p, base, path=BASELINE, min_runs=3, note=""):
    """Write the set's scenarios into the baseline (the others are kept). Refuses a set with
    a run that failed its scenario check, or fewer than min_runs runs of a scenario."""
    bad = [r["dir"] for rs in p["by_run"].values() for r in rs if not r["passed"]]
    few = [f"{k} ({len(v)})" for k, v in p["by_run"].items() if len(v) < min_runs]
    if bad or few:
        raise SystemExit("score_set: not accepted: " + "; ".join(
            ([f"runs that failed their scenario check: {', '.join(bad)}"] if bad else []) +
            ([f"fewer than {min_runs} runs: {', '.join(few)}"] if few else [])))
    shas = {k: {r["scenario_sha"] for r in v} for k, v in p["by_run"].items()}
    mixed = [k for k, v in shas.items() if len(v) > 1]
    if mixed:
        raise SystemExit(f"score_set: not accepted: runs of different scenario files: {', '.join(mixed)}")
    now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    commit = git_commit()
    base = json.loads(json.dumps(base))
    base.setdefault("schema", 1)
    base["about"] = ("The accepted SITL baseline (roadmap R2.7b): per scenario, each accepted run's metrics "
                     "(tools/score_set.py METRICS). Change it only with tools/score_set.py --accept, in a PR.")
    for name, runs in p["by_run"].items():
        s = p["scenarios"][name]
        base["scenarios"][name] = {
            "accepted_utc": now, "accepted_at_commit": commit, "note": note, "scenario_sha": runs[0]["scenario_sha"],
            "runs": [{"run_dir": r["dir"], "started_utc": r["started"], "gdn_version": r["version"],
                      "values": {k: (None if not math.isfinite(v) else round(v, 6)) for k, v in values(r["metrics"]).items()}}
                     for r in runs],
            "pooled": {"cep95_5min_m": s["windows"]["300"]["cep95_m"], "cep95_10min_m": s["windows"]["600"]["cep95_m"],
                       "flights_under_max_5min": s["windows"]["300"]["flights_under_max"],
                       "over3sigma_pct": s.get("over3sigma_pct"), "alt_p95_m": s.get("alt_p95_m")}}
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(base, indent=1, sort_keys=True) + "\n")
    return base


# --- the set's report ---
def _f(v, nd=1):
    return "n/a" if v is None or (isinstance(v, float) and not math.isfinite(v)) else f"{v:.{nd}f}"


def report_md(p, rows, notes, base_path):
    ok = lambda b: "PASS" if b else "FAIL"  # noqa: E731
    t = p["total"]
    L = ["# SITL set: scores and the baseline", "",
         f"{t['runs']} runs of {t['scenarios']} scenarios, pooled per scenario as the requirements count. Run "
         "directories are listed in set.json.", "", "![set](set.png)", "", "## Against the requirements", "",
         "| Scenario | Runs | 5 min CEP95 (pooled) | 5 min flights never over the max | 10 min CEP95 | "
         "over 3 sigma (REQ-INT-001) | altitude p95 (REQ-NAV-010) |", "|---|---|---|---|---|---|---|"]
    for name, s in p["scenarios"].items():
        w5, w10 = s["windows"]["300"], s["windows"]["600"]
        L.append(f"| {name} ({s['requirement']}) | {s['runs']} ({s['flight_min']:.0f} min) | {_f(w5['cep95_m'])} m of "
                 f"{w5['cep95_limit_m']:g}: {ok(w5['cep95_ok'])} | {w5['flights_under_max']} of {w5['flights']} under "
                 f"{w5['max_limit_m']:g} m: {ok(w5['max_rule_ok'])} | "
                 + (f"{_f(w10['cep95_m'])} m of {w10['cep95_limit_m']:g} ({w10['complete']} of {w10['flights']} "
                    f"complete)" if w10["samples"] else "not flown") + " | "
                 f"{_f(s.get('over3sigma_pct'), 2)}% of {s['over3sigma_limit_pct']:g}: "
                 f"{ok(s.get('over3sigma_pct', 1e9) <= s['over3sigma_limit_pct'])} | "
                 f"{_f(s.get('alt_p95_m'))} m of {s['alt_limit_m']:g}: {ok(s.get('alt_p95_m', 1e9) <= s['alt_limit_m'])} |")
    L += ["", "The max rule asks 99% of flights; a few flights can only show all of them under, or not.", ""]
    acq = {k: s["acquisition"] for k, s in p["scenarios"].items() if "acquisition" in s}
    if acq:
        for k, a in acq.items():
            L.append(f"- **REQ-ACQ-001, {k}:** {a['ok']} of {a['flights']} flights under {a['limit_m']:g} m at "
                     f"{a['at_s']:g} s (median {_f(a['error_median_m'])} m).")
    if "judged" in t:
        L += ["", "## Integrity (proxy until R4.4)", "",
              "GDN \"flags\" while 3 x its sigma is over the class's 5-minute max limit. A lost-lock case is the "
              "error passing 3 x that limit; it's detected if flagged within 30 s. False alarms count on nominal "
              "flights (no sensor faults, no start error).", "",
              "| Scenario | Lost-lock cases flagged in 30 s (REQ-INT-002, 99%) | False alarms, nominal (REQ-INT-003, "
              "1 per 10 h) | Time flagged (median run) |", "|---|---|---|---|"]
        for name, s in p["scenarios"].items():
            ig = s.get("integrity")
            if ig:
                L.append(f"| {name} | {ig['detected']} of {ig['judged']} | "
                         + (f"{ig['false_alarms']} in {ig['nominal_h'] * 60:.0f} min" if ig["nominal_h"] else "not nominal")
                         + f" | {_f(ig['flag_pct_median'])}% |")
        L.append(f"| **all** | **{t['detected']} of {t['judged']} ({_f(t['detection_pct'])}%)** | "
                 f"**{t['false_alarms']} in {t['nominal_h']:.2f} h ({_f(t['false_per_10h'])} per 10 h)** | |")
    L += ["", "## Cost", "", "| Scenario | eskf_node CPU mean (worst run) | memory max | processing p95 | "
          "slowest 30 s real-time factor |", "|---|---|---|---|---|"]
    for name, s in p["scenarios"].items():
        c = s["cost"]
        L.append(f"| {name} | {_f(c['cpu_mean_max_pct'])}% of one core | {_f(c['rss_max_mb'])} MB | "
                 f"{_f(c['proc_p95_max_ms'], 3)} ms | {_f(c['rtf_min'], 3)} |")
    L += ["", f"## Against the baseline ({_rel(base_path)})", ""]
    if rows:
        g = [r for r in rows if r["gated"]]
        worse = [r for r in g if r["verdict"] == "WORSE"]
        L.append(f"Gated ({len(g)} comparisons): **{len(worse)} worse**, "
                 f"{sum(r['verdict'] == 'BETTER' for r in g)} better, {sum(r['verdict'] == 'within' for r in g)} "
                 "within the tolerance.")
        L += ["", "| Scenario | Metric | Baseline runs (best to worst) | This set (median) | x the worst | Verdict |",
              "|---|---|---|---|---|---|"]
        for gated in (True, False):
            if not gated:
                L += ["", "Reported, not gated (SITL's run-to-run spread is wider than a regression could show, "
                      "until GDN's drift is bounded):", "",
                      "| Scenario | Metric | Baseline runs (best to worst) | This set (median) | x the worst | "
                      "Where |", "|---|---|---|---|---|---|"]
            for r in (x for x in rows if x["gated"] == gated):
                u = f" {r['unit']}" if r["unit"] else ""
                nd = 3 if r["metric"] in ("proc_p95", "nees_dev") else 1
                lo, hi = (r["base_min"], r["base_max"]) if r["better"] == "lower" else (r["base_max"], r["base_min"])
                v = f"**{r['verdict']}**" if r["verdict"] in ("WORSE", "BETTER") else r["verdict"]
                L.append(f"| {r['scenario']} | {r['label']} | {_f(lo, nd)} to {_f(hi, nd)}{u} ({r['base_runs']}) | "
                         f"{_f(r['new_median'], nd)}{u} ({r['new_runs']}) | {_f(r['x_worst'], 2)} | {v} |")
    else:
        L.append("Nothing to compare.")
    sp = loo_spread(p["by_run"])
    if sp:
        L += ["", "## Run-to-run spread (leave one out)", "",
              "Each run against the worst of its scenario's other runs in this set: how far an unchanged GDN "
              "lands past a baseline of the others. A gated metric's tolerance has to sit above it.", "",
              "| Metric | Comparisons | Past the worst | Median x the worst | Max x the worst | Tolerance |",
              "|---|---|---|---|---|---|"]
        for k, x in sp.items():
            L.append(f"| {x['label']} | {x['n']} | {x['over_1']} | {_f(x['median'], 2)} | {_f(x['max'], 2)} | "
                     + (f"{x['tolerance_x']:.2f} x (gated)" if x["gated"] else "reported") + " |")
    for n in notes:
        L.append(f"- {n}")
    return "\n".join(L) + "\n"


def plot(p, base, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.titlesize": 10, "axes.titleweight": "bold",
                         "axes.titlelocation": "left", "legend.frameon": False})
    names = list(p["scenarios"])
    y = np.arange(len(names))[::-1]
    fig, axs = plt.subplots(1, 3, figsize=(16, 1.6 + 0.42 * len(names)), sharey=True)
    fig.subplots_adjust(left=0.2, right=0.98, wspace=0.08, top=0.84, bottom=0.18)
    panels = [("max_5min", "5-minute max error (m, log)", True), ("over3sigma", "samples over 3 sigma (%)", False),
              ("cpu_mean", "eskf_node CPU mean (% of one core)", False)]
    for ax, (key, title, logx) in zip(axs, panels):
        for yy, name in zip(y, names):
            b = base["scenarios"].get(name)
            if b:
                bv = [r["values"].get(key) for r in b["runs"] if r["values"].get(key) is not None]
                if bv:
                    ax.plot([min(bv), max(bv)], [yy, yy], color=MUTED, lw=6, alpha=0.35, solid_capstyle="round",
                            label="baseline runs (range)" if yy == y[0] else None)
            nv = [values(r["metrics"])[key] for r in p["by_run"][name]]
            nv = [v for v in nv if math.isfinite(v)]
            ax.plot(nv, [yy] * len(nv), "o", ms=5, color=BLUE, mec="white", mew=1,
                    label="this set's runs" if yy == y[0] else None)
            if key == "max_5min":
                lim = p["scenarios"][name]["windows"]["300"]["max_limit_m"]
                ax.plot([lim], [yy], "|", ms=14, mew=2, color=RED, label="limit" if yy == y[0] else None)
            if key == "over3sigma":
                ax.axvline(p["scenarios"][name]["over3sigma_limit_pct"], color=RED, lw=1)
        if logx:
            ax.set_xscale("log")
        else:
            ax.set_xlim(left=0)
        ax.set_title(title)
        ax.grid(color=GRID, lw=0.5, axis="x")
        ax.set_ylim(-0.7, len(names) - 0.3)
    axs[0].set_yticks(y)
    axs[0].set_yticklabels(names)
    axs[0].legend(loc="upper center", bbox_to_anchor=(1.6, -0.08), ncol=3, fontsize=8)
    t = p["total"]
    fig.suptitle(f"SITL set: {t['runs']} runs of {t['scenarios']} scenarios against the requirements and the accepted "
                 "baseline", x=0.02, ha="left", fontsize=11, fontweight="bold", color=INK)
    fig.savefig(out_png, dpi=90, facecolor="white")
    plt.close(fig)


# --- self-check ---
def selfcheck():
    ok = True

    def ck(c, what):
        nonlocal ok
        print(f"{'PASS' if c else 'FAIL'}  selfcheck: {what}")
        ok &= bool(c)
    ck(judge([100, 120, 150], [160], "lower", 0.1, 2)[0] == "within", "10 m past the worst of 150 m: within 10%")
    ck(judge([100, 120, 150], [170, 180, 200], "lower", 0.1, 2)[0] == "WORSE", "the median 180 m past 165 m: WORSE")
    ck(judge([100, 120, 150], [80], "lower", 0.1, 2)[0] == "BETTER", "80 m under the best of 100 m: BETTER")
    ck(judge([50, 60], [35], "higher", 0.0, 10)[0] == "WORSE", "a detection rate 15 points under the worst: WORSE")
    ck(judge([float("nan")], [1.0], "lower", 0, 1)[0] == "n/a", "nothing in the baseline: n/a")

    # Pooling: two flights, sample errors 1..100 and 101..200 m at 1 Hz: pooled CEP95 190.05 m.
    def run(name, errs, max_ok, passed=True):
        t = np.arange(len(errs), dtype=float)
        m = {"scenario": name, "vehicle": "x500", "requirement": "REQ-NAV-001", "map_condition": "x", "route_s": t[-1],
             "windows": {w: {"complete": False, "max_ok": max_ok, "max_m": float(max(errs)), "cep95_m": 0.0,
                             "cep95_limit_m": 15.0, "max_limit_m": 30.0} for w in WINDOWS},
             "consistency": {"limit_pct": 1.0, "over3sigma_pct": 50.0, "nees_mean": 20.0},
             "acquisition": {"judged": False},
             "integrity": {"judged": 2, "detected": 1, "flight_h": 0.5, "nominal": True, "false_alarms": 1,
                           "flag_pct": 10.0, "req_int_002_pct": 99.0, "req_int_003_per_10h": 1.0}}
        s = {"t": t, "err": np.asarray(errs, float), "sig": np.full(len(t), 40.0), "nees": np.full(len(t), 2.0),
             "fly": np.ones(len(t), bool), "verr": np.full(len(t), 1.0)}
        return {"dir": name, "metrics": m, "series": s, "passed": passed, "version": "x", "started": "x",
                "scenario_sha": "abc"}
    rs = [run("a", np.arange(1.0, 101.0), True), run("a", np.arange(101.0, 201.0), False)]
    p = pool(rs)
    a = p["scenarios"]["a"]
    ck(abs(a["windows"]["300"]["cep95_m"] - np.percentile(np.arange(1.0, 201.0), 95)) < 1e-9 and
       a["windows"]["300"]["flights_under_max"] == 1 and not a["windows"]["300"]["max_rule_ok"],
       f"pooled CEP95 over both flights' samples ({a['windows']['300']['cep95_m']:.2f} m), the max rule 1 of 2")
    ck(abs(a["over3sigma_pct"] - 100 * np.mean(np.arange(1.0, 201.0) > 120)) < 1e-9, "pooled share over 3 sigma")
    ck(p["total"]["judged"] == 4 and p["total"]["detected"] == 2 and abs(p["total"]["false_per_10h"] - 20) < 1e-9,
       "integrity pooled: 2 of 4 cases, 2 false alarms in 1 h = 20 per 10 h")
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        bp = Path(td) / "b.json"
        try:
            accept(p, read_baseline(bp), bp, min_runs=3)
            ck(False, "accept refuses fewer than 3 runs")
        except SystemExit:
            ck(True, "accept refuses fewer than 3 runs")
        rs2 = rs + [run("a", np.arange(1.0, 101.0), True, passed=False)]
        try:
            accept(pool(rs2), read_baseline(bp), bp, min_runs=3)
            ck(False, "accept refuses a run that failed its scenario check")
        except SystemExit:
            ck(True, "accept refuses a run that failed its scenario check")
        b = accept(pool(rs), read_baseline(bp), bp, min_runs=2)
        rows, notes = compare(pool(rs), b)
        ck(bp.exists() and rows and all(r["verdict"] in ("within", "among") for r in rows) and not notes,
           "the accepted set compared with itself: all within")
        worse = [run("a", np.arange(1.0, 101.0) * 3, False)] * 2
        rows, _ = compare(pool(worse), b)
        ck(any(r["metric"] == "max_5min" and r["verdict"] == "above all" and abs(r["x_worst"] - 1.5) < 1e-9
               for r in rows) and not any(r["verdict"] == "WORSE" for r in rows),
           "three times the error: above all the baseline's runs (300 m, 1.5 x the worst 200 m), reported (not gated)")
        hot = [dict(r, metrics=dict(r["metrics"], cpu={"mean_pct": 9.0, "rss_max_mb": 40.0})) for r in rs]
        b2 = json.loads(json.dumps(b))
        for x in b2["scenarios"]["a"]["runs"]:
            x["values"]["cpu_mean"] = 3.0
        rows, _ = compare(pool(hot), b2)
        ck(any(r["metric"] == "cpu_mean" and r["verdict"] == "WORSE" for r in rows), "CPU 9% against 3%: WORSE (gated)")
        sp = loo_spread(pool(rs)["by_run"])
        ck(abs(sp["max_5min"]["max"] - 2.0) < 1e-9, f"leave-one-out spread: 200 m against 100 m is 2 x "
           f"({sp['max_5min']['max']})")
        other = [dict(r, scenario_sha="def") for r in rs]
        _, notes = compare(pool(other), b)
        ck(any("changed" in n for n in notes), "a changed scenario file isn't compared")
    print("SELFCHECK PASSED" if ok else "SELFCHECK FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("runs", nargs="*", help="scored run directories, or directories holding them")
    ap.add_argument("--out", help="where set.md, set.json and set.png go (default: the first argument if a "
                                  "directory of runs, else the current directory)")
    ap.add_argument("--baseline", default=str(BASELINE))
    ap.add_argument("--accept", action="store_true", help="make this set the baseline for its scenarios")
    ap.add_argument("--min-runs", type=int, default=3, help="--accept: runs needed per scenario")
    ap.add_argument("--note", default="", help="--accept: why (kept in the baseline)")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--selfcheck", action="store_true")
    a = ap.parse_args()
    if a.selfcheck:
        return 0 if selfcheck() else 1
    if not a.runs:
        ap.error("give run directories or --selfcheck")
    runs = load(a.runs)
    if not runs:
        raise SystemExit("score_set: no scored runs found (metrics.json: tools/score_run.py)")
    missing = [r["dir"] for r in runs if r["series"] is None]
    if missing:
        raise SystemExit("score_set: runs without series.npz (score them again with tools/score_run.py): "
                         + ", ".join(missing))
    p = pool(runs)
    base = read_baseline(a.baseline)
    if a.accept:
        base = accept(p, base, a.baseline, a.min_runs, a.note)
        print(f"score_set: accepted {len(p['by_run'])} scenarios into {_rel(a.baseline)}: commit it in a PR")
    rows, notes = compare(p, base)
    out = Path(a.out) if a.out else (Path(a.runs[0]) if len(a.runs) == 1 and not (Path(a.runs[0]) / "metrics.json")
                                     .exists() else Path("."))
    out.mkdir(parents=True, exist_ok=True)
    doc = {k: v for k, v in p.items() if k != "by_run"}
    doc["runs"] = {k: [r["dir"] for r in v] for k, v in p["by_run"].items()}
    doc["comparison"] = rows
    doc["notes"] = notes
    (out / "set.json").write_text(json.dumps(doc, indent=1) + "\n")
    (out / "set.md").write_text(report_md(p, rows, notes, a.baseline))
    if not a.no_plot:
        plot(p, base, out / "set.png")
    print(summary(p))
    worse = [r for r in rows if r["verdict"] == "WORSE"]
    above = [r for r in rows if r["verdict"] == "above all"]
    print(f"baseline: {sum(r['gated'] for r in rows)} gated comparisons, {len(worse)} worse; reported: {len(above)} of "
          f"{sum(not r['gated'] for r in rows)} above all the baseline's runs"
          + "".join(f"\n  WORSE {r['scenario']} {r['label']}: {r['new_median']:.3g} against the baseline's "
                    f"{r['base_min']:.3g} to {r['base_max']:.3g}" for r in worse))
    for n in notes:
        print(f"baseline: {n}")
    print(f"score_set: {out / 'set.md'}")
    return 1 if worse else 0


if __name__ == "__main__":
    sys.exit(main())
