#!/usr/bin/env python3
"""Score one SITL run, and write its short report (roadmap R2.7a, TEST-042; integrity R2.7b).

    python3 tools/score_run.py RUN_DIR                  # metrics.json, series.npz, report.md, report.png
    python3 tools/score_run.py RUN_DIR [RUN_DIR ...]    # each, then the set (tools/score_set.py)
    python3 tools/score_run.py --selfcheck              # the metrics on made-up data (CI)

RUN_DIR is a tools/run_scenario.py run (it calls this after its check). Needs ROS 2 for the
bag and GDN's run log (the dev image); --selfcheck doesn't.

The clock (agreed): GNSS-free time counts from the route's start (/gdn/sim/route_start,
cruise height), the clock of the scenario's GNSS cuts, spoofing and faults. GDN uses no
GNSS in SITL at all, so the whole flight is GNSS-free to it; the score windows are
[route start, route start + 300 s] and [.., + 600 s], each cut at the landing.

From GDN's run log (its state after every input, at 10 Hz here) against the truth
(Gazebo's pose, /gdn/sim/truth_odom in the bag):
  - accuracy: horizontal error CEP50, CEP95 and max per window, against the vehicle
    class's REQ-NAV limits read from docs/requirements.md (x500 multirotor REQ-NAV-001,
    standard_vtol quadplane REQ-NAV-002, rc_cessna fixed-wing REQ-NAV-003). One run
    scores its own CEP95 and max; the requirement pools a set of runs (CEP95 over all
    samples, the max rule over flights), which the multi-run form prints;
  - vertical (REQ-NAV-010): altitude error p95, GDN's NED down against the truth's;
  - consistency (REQ-INT-001): the share of samples whose horizontal error is over 3 x
    GDN's published sigma (sqrt(P_nn + P_ee)), as mc_campaign; NEES, the 2-D
    err' P_h^-1 err (mean, expect 2; share over 9, expect 1.1%);
  - NIS per aiding sensor, from GDN's own update records: mean against the
    innovation's dimension (its expected value), share over the chi-square 99.7% point;
  - time to acquire: from the route's start to when the error goes under the class's
    5-minute max limit and stays under it for 30 s (REQ-ACQ-001 asks within 120 s, from
    up to 500 m; judged only when the scenario asks a start error);
  - CPU and memory: eskf_node's, sampled once a second by the runner (cpu.csv: share of
    one core, resident MB);
  - latency, per IMU sample, from GDN's run log: its processing (state written minus
    input received, both on the host's clock: the headline) and sample to output (state
    written minus PX4's sample time). In SITL PX4's clock is Gazebo's simulated time,
    synced to the host's over uXRCE-DDS: when Gazebo runs slower than real time PX4's
    stamps fall behind between re-syncs, so sample to output then measures that drift,
    not a latency. Even at a real-time factor of 0.998 it spread from 1 us to 300 ms
    (TEST-042), so in SITL it's a diagnostic; the real transport latency is measured on
    the hardware (R6.3). Hence:
  - integrity (REQ-INT-002, 003), a labelled proxy until R4.4 gives GDN a status (agreed,
    R2.7b): GDN "flags" while 3 x its sigma is over the class's 5-minute max limit. A
    case (REQ-INT-002) starts when the error passes 3 x that limit, having been under the
    limit since the last case; it's detected if the flag is on within 30 s. A false alarm
    (REQ-INT-003) is a flag episode (off for under 10 s doesn't end one) in which the
    error never passes the limit: a warning that comes true isn't false. The rate counts
    nominal flights only (no sensor faults, no start error: GDN uses no GNSS, so
    GNSS cuts and spoofing are nominal to it);
  - the simulation's real-time factor (Gazebo's time against the wall clock, from the
    truth's stamps and the bag's receive times; TEST-042: 1.000 on a quiet PC, 0.86 on
    battery in power-saver): below RTF_TIMING_MIN the sample-to-output latency is marked
    unreliable.

The map condition (REQ-DB-002): the R2.2 sites run GDN on an independent map (GLO-30)
against a swisstopo world, so their scores count; the Zermatt world shares its map and
is a regression check only. series.npz keeps the per-sample error, sigma and altitude
error, so tools/score_set.py can pool a set of runs (and compare it with the accepted
baseline) without ROS.
"""
import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
ROOT = Path(__file__).resolve().parents[1]
CLASS = {"multirotor": ("REQ-NAV-001", "multirotor"), "vtol": ("REQ-NAV-002", "quadplane"),
         "fixedwing": ("REQ-NAV-003", "fixed-wing")}
WINDOWS = (300.0, 600.0)
STATE_HZ = 10.0
ACQ_HOLD_S = 30.0
RTF_TIMING_MIN = 0.98                          # below it, PX4-clocked latency is the sim's drift
INT_DETECT_S = 30.0                            # REQ-INT-002: flagged within 30 s
INT_ERR_FACTOR = 3.0                           # REQ-INT-002: the error passes 3 x the requirement
INT_MERGE_S = 10.0                             # the proxy flag off for less than this: one episode
CHI2_997 = {1: 8.81, 2: 11.62, 3: 13.93}      # chi-square 99.7% points by dimension
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#d9d8d4"
BLUE, ORANGE, GREEN, RED = "#2a78d6", "#eb6834", "#1f9e6a", "#b3261e"


# --- metrics (unit-checked by --selfcheck) ---
def cep(err, q):
    return float(np.percentile(err, q)) if len(err) else float("nan")


def window_scores(t, err, over3, t_end, limits):
    """Per window [0, w] (t from the route's start, cut at t_end): CEP50, CEP95, max,
    the share over 3 sigma, and the class limits."""
    out = {}
    for w in WINDOWS:
        sel = (t >= 0) & (t <= min(w, t_end))
        e = err[sel]
        c95_lim, max_lim = limits.get(int(w), (float("nan"), float("nan")))
        out[str(int(w))] = {
            "window_s": w, "covered_s": float(min(w, t_end)), "complete": bool(t_end >= w), "n": int(sel.sum()),
            "cep50_m": cep(e, 50), "cep95_m": cep(e, 95), "max_m": float(e.max()) if len(e) else float("nan"),
            "over3sigma_pct": float(100.0 * over3[sel].mean()) if sel.any() else float("nan"),
            "cep95_limit_m": c95_lim, "max_limit_m": max_lim,
            "cep95_ok": bool(len(e) and cep(e, 95) <= c95_lim), "max_ok": bool(len(e) and e.max() <= max_lim)}
    return out


def nees2(err_ne, pnn, pee, pne):
    """2-D NEES err' P^-1 err per sample (inf where P isn't positive definite)."""
    det = pnn * pee - pne * pne
    en, ee = err_ne[:, 0], err_ne[:, 1]
    with np.errstate(divide="ignore", invalid="ignore"):
        v = (pee * en * en - 2 * pne * en * ee + pnn * ee * ee) / det
    v[~(det > 0) | ~(pnn > 0)] = np.inf
    return v


def time_to_acquire(t, err, limit, hold=ACQ_HOLD_S):
    """First time (t >= 0) from which the error stays under limit for hold seconds; None."""
    sel = t >= 0
    t, err = t[sel], err[sel]
    under = err <= limit
    start = None
    for k in range(len(t)):
        if under[k]:
            if start is None:
                start = k
            if t[k] - t[start] >= hold:
                return float(t[start])
        else:
            start = None
    return None


def flag_episodes(t, flag, merge_s=INT_MERGE_S):
    """(start, end) sample indices of the flag's episodes, end inclusive; an episode isn't
    ended by the flag going off for less than merge_s."""
    ep = []
    k, n = 0, len(t)
    while k < n:
        if not flag[k]:
            k += 1
            continue
        a = k
        while k < n and flag[k]:
            k += 1
        if ep and t[a] - t[ep[-1][1]] < merge_s:
            ep[-1] = (ep[-1][0], k - 1)
        else:
            ep.append((a, k - 1))
    return ep


def integrity(t, err, sig, limit, detect_s=INT_DETECT_S, factor=INT_ERR_FACTOR, merge_s=INT_MERGE_S):
    """The integrity proxy (until R4.4) on one flight's samples (t from the route's start):
    flag = 3 sigma > limit. Cases: the error passes factor x limit, having been under limit
    since the last case (or the start); detected if the flag is on at some sample in
    [crossing, crossing + detect_s]; a case the flight ended before judging (and not yet
    detected) isn't judged. False alarms: flag episodes (flag_episodes) in which the error
    never passes the limit."""
    t, err, sig = (np.asarray(x, dtype=float) for x in (t, err, sig))
    flag = 3.0 * sig > limit
    cases, armed = [], True
    for k in range(len(t)):
        if armed and err[k] > factor * limit:
            on = np.nonzero(flag & (t >= t[k]) & (t <= t[k] + detect_s))[0]
            done = bool(len(on)) or bool(t[-1] >= t[k] + detect_s)
            cases.append({"t_s": float(t[k]), "error_m": float(err[k]), "judged": done, "detected": bool(len(on)),
                          "latency_s": float(t[on[0]] - t[k]) if len(on) else None})
            armed = False
        elif not armed and err[k] <= limit:
            armed = True
    ep = flag_episodes(t, flag, merge_s)
    false = [a for a, b in ep if err[a:b + 1].max() <= limit]
    judged = [c for c in cases if c["judged"]]
    return {"limit_m": float(limit), "case_error_m": float(factor * limit), "detect_s": float(detect_s),
            "flag_pct": float(100 * flag.mean()) if len(t) else float("nan"),
            "flag_while_under_pct": float(100 * (flag & (err <= limit)).mean()) if len(t) else float("nan"),
            "cases": cases, "judged": len(judged), "detected": sum(c["detected"] for c in judged),
            "detection_pct": float(100 * np.mean([c["detected"] for c in judged])) if judged else float("nan"),
            "episodes": len(ep), "false_alarms": int(len(false)), "false_alarm_t_s": [float(t[k]) for k in false],
            "flight_h": float((t[-1] - t[0]) / 3600.0) if len(t) > 1 else 0.0}


def nis_stats(nis, dim):
    nis = np.asarray(nis, dtype=float)
    nis = nis[np.isfinite(nis)]
    if not len(nis):
        return None
    lim = CHI2_997.get(dim, 9.0)
    return {"n": int(len(nis)), "dim": int(dim), "mean": float(nis.mean()), "expected": float(dim),
            "median": float(np.median(nis)), "over_997_pct": float(100.0 * (nis > lim).mean()), "expected_pct": 0.3}


# --- reading a run ---
def read_run(run):
    """What scoring needs from one run directory (needs ROS: the dev image)."""
    import check_scenario as c
    import check_sim_sensors as cs
    import scenario as scn
    run = Path(run)
    sc = scn.parse(__import__("yaml").safe_load((run / "scenario.yaml").read_text()))
    res = json.loads((run / "result.json").read_text())
    t0 = c.route_start_time(run)
    t_land = c.landed_time(run / "fly.log")
    truth = c.read_truth(run / "sitl_bag")
    vtruth = cs.read_bag(run / "sitl_bag", ["/gdn/truth"])[1]["/gdn/truth"]       # adapter: NED, z down from the pad's map height
    vt = np.array([(t, m.point.z) for t, m in vtruth]) if vtruth else np.zeros((0, 2))
    st, upd, imu_lat = [], [], []
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(res["run_log"]), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {x.name: get_message(x.type) for x in r.get_all_topics_and_types()}
    last_st = -1.0
    pending = None                      # (record time, PX4 sample time) of the last IMU input
    while r.has_next():
        topic, data, t_ns = r.read_next()
        t_rec = t_ns * 1e-9
        if topic == "/fmu/out/sensor_combined":
            pending = (t_rec, deserialize_message(data, types[topic]).timestamp * 1e-6)
        elif topic == "/gdn/log/state":
            if pending is not None:
                imu_lat.append((pending[0], t_rec - pending[0], t_rec - pending[1]))
                pending = None
            if t_rec - last_st >= 1.0 / STATE_HZ:
                m = deserialize_message(data, types[topic])
                last_st = t_rec
                st.append((t_rec, m.position[0], m.position[1], m.position[2], m.p_diag[0], m.p_diag[1], m.p_ne,
                           m.p_diag[2], m.flags))
        elif topic == "/gdn/log/update":
            m = deserialize_message(data, types[topic])
            if m.reason == 0 and len(m.innovation):
                upd.append((t_rec, m.sensor, len(m.innovation), m.nis))
        elif not topic.startswith("/gdn/log/"):
            pending = None              # another input: its state isn't the IMU's
    cpu = np.genfromtxt(run / "cpu.csv", delimiter=",", names=True) if (run / "cpu.csv").exists() else None
    return {"run": run, "scenario": sc, "result": res, "t0": t0, "t_land": t_land, "truth": truth, "vtruth": vt,
            "state": np.array(st), "updates": upd, "imu_lat": np.array(imu_lat), "cpu": cpu}


SENSOR_NAMES = {1: "TRN", 2: "AGL", 3: "baro", 4: "mag (3-axis)", 5: "mag heading", 6: "ZUPT", 7: "GNSS"}


def score(d):
    import sim_vehicles
    from score_requirements import DOC, read_limits
    sc = d["scenario"]
    kind = sim_vehicles.get(sc["vehicle"]).kind
    req, cls = CLASS[kind]
    lim = read_limits(DOC.read_text())
    t0 = d["t0"]
    s = d["state"]
    tr = d["truth"]
    t_end = (d["t_land"] or s[-1, 0]) - t0
    t = s[:, 0] - t0
    tn, te = np.interp(s[:, 0], tr[:, 0], tr[:, 2]), np.interp(s[:, 0], tr[:, 0], tr[:, 1])
    err_ne = np.stack([s[:, 1] - tn, s[:, 2] - te], 1)
    err = np.hypot(err_ne[:, 0], err_ne[:, 1])
    sig = np.sqrt(np.maximum(s[:, 4] + s[:, 5], 0))
    over3 = err > 3 * sig
    nees = nees2(err_ne, s[:, 4], s[:, 5], s[:, 6])
    fly = (t >= 0) & (t <= t_end)
    out = {"scenario": sc["name"], "run_dir": str(d["run"]), "vehicle": sc["vehicle"], "class": cls,
           "requirement": req, "world": sc["world"],
           "map_condition": "same map (regression only)" if sc["world"] == "zermatt" else "independent map (scored)",
           "route_s": float(t_end), "start_error_m": sc["start_error"]["distance_m"],
           "gnss": sc["gnss"]["loss"], "faults": len(sc["faults"])}
    out["windows"] = window_scores(t, err, over3, t_end, lim.get(req, {}))
    out["whole_route"] = {"cep50_m": cep(err[fly], 50), "cep95_m": cep(err[fly], 95),
                          "max_m": float(err[fly].max()) if fly.any() else float("nan"),
                          "final_m": float(err[fly][-1]) if fly.any() else float("nan"),
                          "rms_m": float(np.sqrt(np.mean(err[fly] ** 2))) if fly.any() else float("nan")}
    # Cross-check: the run's own replay_scorer (/gdn/odom against /gdn/truth, whole run).
    sl = d["run"] / "scorer.log"
    mm = re.findall(r"RMS whole run: ([\d.]+) m \| P95: ([\d.]+) m \| MAX: ([\d.]+) m", sl.read_text()) if sl.exists() else []
    if mm:
        out["replay_scorer_whole_run"] = {"rms_m": float(mm[-1][0]), "p95_m": float(mm[-1][1]), "max_m": float(mm[-1][2])}
    ne = nees[fly]
    out["consistency"] = {"over3sigma_pct": float(100 * over3[fly].mean()) if fly.any() else float("nan"),
                          "limit_pct": lim.get("REQ-INT-001", 1.0),
                          "nees_mean": float(np.mean(np.minimum(ne, 1e6))) if len(ne) else float("nan"),
                          "nees_median": float(np.median(ne)) if len(ne) else float("nan"),
                          "nees_over9_pct": float(100 * (ne > 9).mean()) if len(ne) else float("nan")}
    out["consistency"]["ok"] = bool(out["consistency"]["over3sigma_pct"] <= out["consistency"]["limit_pct"])
    verr = np.full(len(t), np.nan)
    # Vertical: GDN's NED down against the adapter's truth (both from the pad's map height).
    vt = d["vtruth"]
    if len(vt):
        vz = np.interp(s[fly, 0], vt[:, 0], vt[:, 1])
        ve = np.abs(s[fly, 3] - vz)
        verr[fly] = ve
        out["vertical"] = {"p95_m": cep(ve, 95), "max_m": float(ve.max()) if len(ve) else float("nan"),
                           "limit_m": lim.get("REQ-NAV-010", float("nan"))}
        out["vertical"]["ok"] = bool(out["vertical"]["p95_m"] <= out["vertical"]["limit_m"])
    # NIS per sensor, over the flight.
    nis = {}
    for name_id in sorted({u[1] for u in d["updates"]}):
        rows = [u for u in d["updates"] if u[1] == name_id and t0 <= u[0] <= t0 + t_end]
        if rows:
            st_ = nis_stats([u[3] for u in rows], rows[0][2])
            if st_:
                nis[SENSOR_NAMES.get(name_id, str(name_id))] = st_
    out["nis"] = nis
    # Acquisition.
    max5 = lim.get(req, {}).get(300, (float("nan"), float("nan")))[1]
    tta = time_to_acquire(t, err, max5)
    acq = {"limit_m": max5, "time_to_acquire_s": tta, "hold_s": ACQ_HOLD_S}
    if sc["start_error"]["distance_m"] > 0:
        a_lim = lim.get("REQ-ACQ-001", (500.0, 120.0))
        at = np.interp(a_lim[1], t, err) if t[-1] >= a_lim[1] else float("nan")
        acq.update({"error_at_s": a_lim[1], "error_m": float(at), "judged": True,
                    "ok": bool(np.isfinite(at) and at <= max5)})
    else:
        acq["judged"] = False
    out["acquisition"] = acq
    # Integrity: the labelled proxy (until R4.4), on the flight's samples.
    ig = integrity(t[fly], err[fly], sig[fly], max5, detect_s=lim.get("REQ-INT-002_s", INT_DETECT_S))
    ig["proxy"] = "GDN flags while 3 sigma > the class's 5-minute max limit (R2.7b proxy, until R4.4)"
    ig["nominal"] = bool(not sc["faults"] and sc["start_error"]["distance_m"] == 0)
    ig["req_int_002_pct"] = lim.get("REQ-INT-002", 99.0)
    ig["req_int_003_per_10h"] = lim.get("REQ-INT-003", 1.0)
    out["integrity"] = ig
    # The simulation's real-time factor, over the flight (30 s windows).
    if len(tr) > 10:
        wall, sim = tr[:, 0], tr[:, 9]
        k = np.searchsorted(wall, wall[0] + np.arange(0.0, wall[-1] - wall[0], 30.0))
        k = k[k < len(wall)]
        if len(k) > 2:
            rtf = np.diff(sim[k]) / np.diff(wall[k])
            out["sim"] = {"real_time_factor": float((sim[k[-1]] - sim[k[0]]) / (wall[k[-1]] - wall[k[0]])),
                          "rtf_min_30s": float(rtf.min()), "timing_reliable": bool(rtf.min() >= RTF_TIMING_MIN)}
    # CPU and memory.
    cpu = d["cpu"]
    if cpu is not None and cpu.size:
        c = np.atleast_1d(cpu)
        cs_ = (c["t"] >= d["t0"]) & (c["t"] <= d["t0"] + t_end)
        cc = c[cs_] if cs_.any() else c
        out["cpu"] = {"mean_pct": float(np.mean(cc["cpu_pct"])), "p95_pct": cep(cc["cpu_pct"], 95),
                      "max_pct": float(np.max(cc["cpu_pct"])), "rss_max_mb": float(np.max(cc["rss_mb"])),
                      "samples": int(len(cc))}
    # Latency.
    la = d["imu_lat"]
    if len(la):
        sel = (la[:, 0] >= t0) & (la[:, 0] <= t0 + t_end)
        p, s2o = la[sel, 1] * 1e3, la[sel, 2] * 1e3
        # PX4's clock is synced to the host's over uXRCE-DDS; when that sync resets ("time
        # jump detected" in px4.log, e.g. when Gazebo can't keep real time), sample times
        # jump, and those samples are left out of sample-to-output (and counted).
        good = np.abs(s2o) < 1000.0
        bad = int((~good).sum())
        s2o = s2o[good] if good.any() else np.array([np.nan])
        out["latency_ms"] = {"clock_jump_samples": bad, "processing_p50": cep(p, 50), "processing_p95": cep(p, 95),
                             "processing_max": float(p.max()), "sample_to_output_p50": cep(s2o, 50),
                             "sample_to_output_p95": cep(s2o, 95), "sample_to_output_max": float(s2o.max()),
                             "n": int(sel.sum())}
    return out, {"t": t, "err": err, "sig": sig, "nees": nees, "fly": fly, "vz": s[:, 3], "verr": verr}


def save_series(series, path):
    """The per-sample series a set is pooled from (tools/score_set.py), without ROS."""
    np.savez_compressed(path, **{k: series[k] for k in ("t", "err", "sig", "nees", "fly", "verr")})


def report(m, series, d, out_dir):
    """report.md and report.png for one run."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.titlesize": 10, "axes.titleweight": "bold",
                         "axes.titlelocation": "left", "legend.frameon": False})
    out_dir = Path(out_dir)
    t, err, sig, nees, fly = series["t"], series["err"], series["sig"], series["nees"], series["fly"]
    w5 = m["windows"]["300"]
    fig, axs = plt.subplots(2, 3, figsize=(16, 8.5))
    fig.subplots_adjust(hspace=0.5, wspace=0.38, top=0.86, bottom=0.08, left=0.07, right=0.97)
    ax = axs[0, 0]
    ax.fill_between(t, 0, 3 * sig, color=BLUE, alpha=0.12, lw=0, label="GDN's 3 sigma")
    ig = m.get("integrity")
    if ig:
        fl = (3 * sig > ig["limit_m"]) & fly
        if fl.any():
            ax.fill_between(t, 0, 1, where=fl, transform=ax.get_xaxis_transform(), color=ORANGE, alpha=0.10, lw=0,
                            label="flagged (proxy: 3 sigma over the max limit)")
    ax.plot(t, err, color=BLUE, lw=1.3, label="horizontal error")
    ax.axhline(w5["max_limit_m"], color=RED, lw=1, ls=(0, (3, 2)), label=f"max limit {w5['max_limit_m']:g} m (5 min)")
    ax.axhline(w5["cep95_limit_m"], color=ORANGE, lw=1, ls=(0, (1, 2)), label=f"CEP95 limit {w5['cep95_limit_m']:g} m")
    ax.axvline(0, color=MUTED, lw=0.8)
    ax.axvline(m["route_s"], color=MUTED, lw=0.8)
    ax.set_yscale("symlog", linthresh=10)
    ax.set_ylim(bottom=0)
    ax.set_xlim(min(-20, t.min()), m["route_s"] + 20)
    ax.set_xlabel("time from the route's start (s)")
    ax.set_ylabel("m")
    ax.set_title("Horizontal error against GDN's 3 sigma")
    ax.legend(loc="upper left", fontsize=7.5)
    ax.grid(color=GRID, lw=0.5)
    ax = axs[0, 1]
    e = np.sort(err[fly])
    if len(e):
        ax.plot(e, np.arange(1, len(e) + 1) / len(e) * 100, color=BLUE, lw=1.6)
        for q, colr in ((50, MUTED), (95, ORANGE)):
            v = cep(e, q)
            ax.axvline(v, color=colr, lw=1, ls=(0, (3, 2)))
            ax.text(v, 3 + q * 0.5, f" CEP{q} {v:.0f} m", color=INK2, fontsize=8)
        ax.axvline(w5["cep95_limit_m"], color=RED, lw=1, label=f"CEP95 limit {w5['cep95_limit_m']:g} m")
        ax.set_xscale("log")
    ax.set_xlabel("horizontal error (m, log)")
    ax.set_ylabel("share of route samples under it (%)")
    ax.set_title("Error distribution over the route")
    ax.legend(loc="lower right", fontsize=7.5)
    ax.grid(color=GRID, lw=0.5)
    ax = axs[0, 2]
    nf = np.where(np.isfinite(nees), nees, 1e6)
    ax.plot(t, nf, color=BLUE, lw=0.8)
    ax.axhline(2, color=GREEN, lw=1, label="expected 2")
    ax.axhline(9, color=RED, lw=1, ls=(0, (3, 2)), label="9 (1.1% expected above)")
    ax.set_yscale("log")
    ax.set_xlim(min(-20, t.min()), m["route_s"] + 20)
    ax.set_xlabel("time from the route's start (s)")
    ax.set_ylabel("horizontal NEES")
    c = m["consistency"]
    ax.set_title(f"NEES: mean {c['nees_mean']:.3g}, over 9 {c['nees_over9_pct']:.1f}%")
    ax.legend(loc="lower right", fontsize=7.5)
    ax.grid(color=GRID, lw=0.5)
    ax = axs[1, 0]
    names = list(m["nis"])
    if names:
        y = np.arange(len(names))
        ratio = [m["nis"][n]["mean"] / m["nis"][n]["expected"] for n in names]
        ax.barh(y, ratio, color=BLUE, height=0.5)
        ax.axvline(1, color=GREEN, lw=1)
        for yy, n in zip(y, names):
            ns = m["nis"][n]
            ax.text(max(ratio) * 1.02 if max(ratio) > 0 else 0.02, yy,
                    f"mean {ns['mean']:.2f} (exp. {ns['expected']:g}); over 99.7% pt {ns['over_997_pct']:.1f}%; n={ns['n']}",
                    va="center", fontsize=7.5, color=INK2)
        ax.set_yticks(y)
        ax.set_yticklabels(names)
        ax.set_xlim(0, max(ratio) * 2.6 if max(ratio) > 0 else 1)
    ax.set_xlabel("NIS mean / expected (1 = consistent)")
    ax.set_title("NIS per aiding sensor (applied updates)")
    ax.grid(color=GRID, lw=0.5, axis="x")
    ax = axs[1, 1]
    cpu = d["cpu"]
    if cpu is not None and cpu.size:
        cc = np.atleast_1d(cpu)
        ax.plot(cc["t"] - d["t0"], cc["cpu_pct"], color=BLUE, lw=1.2, label="CPU (% of one core)")
        ax2 = ax.twinx()
        ax2.plot(cc["t"] - d["t0"], cc["rss_mb"], color=MUTED, lw=1, ls=(0, (3, 2)))
        ax2.set_ylabel("resident memory (MB, dashed)", fontsize=8)
        ax2.set_ylim(bottom=0)
    else:
        ax.text(0.5, 0.5, "no CPU samples (cpu.csv: tools/run_scenario.py runs)", ha="center", va="center",
                transform=ax.transAxes, color=INK2)
    ax.set_ylim(bottom=0)
    ax.set_xlabel("time from the route's start (s)")
    ax.set_ylabel("% of one core")
    cp = m.get("cpu", {})
    ax.set_title(f"eskf_node CPU: mean {cp.get('mean_pct', float('nan')):.1f}%, p95 {cp.get('p95_pct', float('nan')):.1f}%")
    ax.grid(color=GRID, lw=0.5)
    ax = axs[1, 2]
    la = d["imu_lat"]
    if len(la):
        sel = (la[:, 0] >= d["t0"]) & (la[:, 0] <= d["t0"] + m["route_s"])
        bins = np.logspace(-3, 3, 61)
        ax.hist(np.maximum(la[sel, 1] * 1e3, 1e-3), bins=bins, color=BLUE, alpha=0.8, label="processing")
        s2o_ = la[sel, 2] * 1e3
        ax.hist(np.maximum(s2o_[np.abs(s2o_) < 1000.0], 1e-3), bins=bins, histtype="step", color=ORANGE, lw=1.5,
                label="PX4 sample to output")
        ax.set_xscale("log")
    lm = m.get("latency_ms", {})
    ax.set_xlabel("ms (log)")
    ax.set_ylabel("IMU samples")
    ax.set_title(f"Latency p95: processing {lm.get('processing_p95', float('nan')):.3f} ms\n"
                 f"(PX4 sample to output {lm.get('sample_to_output_p95', float('nan')):.0f} ms: the sim's clock)",
                 fontsize=9)
    ax.legend(loc="upper right", fontsize=7.5)
    ax.grid(color=GRID, lw=0.5)
    ok = lambda b: "PASS" if b else "FAIL"  # noqa: E731
    fig.suptitle(f"{m['scenario']}: {m['vehicle']} ({m['class']}, {m['requirement']}) over {m['world']}; "
                 f"route {m['route_s']:.0f} s\n5 min: CEP95 {w5['cep95_m']:.1f} m ({ok(w5['cep95_ok'])}), "
                 f"max {w5['max_m']:.1f} m ({ok(w5['max_ok'])}); over 3 sigma {c['over3sigma_pct']:.1f}% "
                 f"({ok(c['ok'])})", x=0.05, ha="left", fontsize=11, fontweight="bold", color=INK)
    fig.savefig(out_dir / "report.png", dpi=90, facecolor="white")
    plt.close(fig)
    (out_dir / "report.md").write_text(report_md(m))


def report_md(m):
    def f(v, nd=1):
        return "n/a" if v is None or (isinstance(v, float) and not math.isfinite(v)) else f"{v:.{nd}f}"

    def verdict(ok, complete=True):
        return ("PASS" if ok else "FAIL") + ("" if complete else " (window not complete)")
    lines = [f"# {m['scenario']}: score", "",
             f"{m['vehicle']} ({m['class']}, {m['requirement']}) over {m['world']} ({m['map_condition']}); "
             f"route {m['route_s']:.0f} s from its start; GNSS to PX4: {m['gnss']}; start error "
             f"{m['start_error_m']:g} m; {m['faults']} sensor faults. GNSS-free time counts from the route's "
             "start (GDN uses no GNSS in SITL).", "", "![score](report.png)", "",
             "| Requirement | Window | Measured | Limit | Verdict |", "|---|---|---|---|---|"]
    for k, w in m["windows"].items():
        lines.append(f"| {m['requirement']} CEP95 | {int(w['window_s']) // 60} min ({w['covered_s']:.0f} s flown) | "
                     f"{f(w['cep95_m'])} m (CEP50 {f(w['cep50_m'])} m) | {w['cep95_limit_m']:g} m | "
                     f"{verdict(w['cep95_ok'], w['complete'])} |")
        lines.append(f"| {m['requirement']} max | {int(w['window_s']) // 60} min | {f(w['max_m'])} m | "
                     f"{w['max_limit_m']:g} m | {verdict(w['max_ok'], w['complete'])} |")
    v = m.get("vertical")
    if v:
        lines.append(f"| REQ-NAV-010 altitude p95 | route | {f(v['p95_m'])} m (max {f(v['max_m'])} m) | "
                     f"{v['limit_m']:g} m | {verdict(v['ok'])} |")
    c = m["consistency"]
    lines.append(f"| REQ-INT-001 over 3 sigma | route | {f(c['over3sigma_pct'], 2)}% | {c['limit_pct']:g}% | "
                 f"{verdict(c['ok'])} |")
    a = m["acquisition"]
    if a["judged"]:
        lines.append(f"| REQ-ACQ-001 error at {a['error_at_s']:g} s | from a {m['start_error_m']:g} m start | "
                     f"{f(a['error_m'])} m | {a['limit_m']:g} m | {verdict(a['ok'])} |")
    ig = m.get("integrity")
    if ig:
        lines.append(f"| REQ-INT-002 lost lock flagged within {ig['detect_s']:g} s (proxy) | error over "
                     f"{ig['case_error_m']:g} m | {ig['detected']} of {ig['judged']} cases | "
                     f"{ig['req_int_002_pct']:g}% | "
                     + (verdict(ig['detection_pct'] >= ig['req_int_002_pct']) if ig['judged'] else "no case") + " |")
        lines.append(f"| REQ-INT-003 false alarms (proxy) | {ig['flight_h'] * 60:.1f} min"
                     f"{'' if ig['nominal'] else ', not nominal (faults or a start error): not counted'} | "
                     f"{ig['false_alarms']} | {ig['req_int_003_per_10h']:g} per 10 h | pooled over a set |")
    lines += ["", "One run's CEP95 and max; the requirement pools a set of runs (tools/score_set.py). The "
              "integrity rows use a proxy until R4.4 gives GDN a status: it \"flags\" while 3 x its sigma is over "
              "the class's 5-minute max limit.", "",
              "| Measure | Value |", "|---|---|",
              f"| whole route | CEP50 {f(m['whole_route']['cep50_m'])} m, CEP95 {f(m['whole_route']['cep95_m'])} m, "
              f"max {f(m['whole_route']['max_m'])} m, RMS {f(m['whole_route']['rms_m'])} m, at the end "
              f"{f(m['whole_route']['final_m'])} m |",
              f"| NEES (2-D) | mean {f(c['nees_mean'], 3)} (expected 2), median {f(c['nees_median'], 2)}, "
              f"over 9 {f(c['nees_over9_pct'])}% (expected 1.1%) |"]
    rs = m.get("replay_scorer_whole_run")
    if rs:
        lines.append(f"| cross-check: replay_scorer (/gdn/odom, whole run incl. take-off and landing) | RMS "
                     f"{f(rs['rms_m'])} m, P95 {f(rs['p95_m'])} m, max {f(rs['max_m'])} m |")
    for n, s in m["nis"].items():
        lines.append(f"| NIS {n} | mean {f(s['mean'], 2)} (expected {s['expected']:g}), over the 99.7% point "
                     f"{f(s['over_997_pct'])}% (expected 0.3%), n={s['n']} |")
    tta = a["time_to_acquire_s"]
    lines.append(f"| time to acquire | {'never' if tta is None else f(tta) + ' s'} (under {a['limit_m']:g} m "
                 f"for {a['hold_s']:g} s) |")
    cp = m.get("cpu")
    if cp:
        lines.append(f"| eskf_node CPU | mean {f(cp['mean_pct'])}%, p95 {f(cp['p95_pct'])}%, max {f(cp['max_pct'])}% "
                     f"of one core; memory max {f(cp['rss_max_mb'])} MB |")
    la = m.get("latency_ms")
    if la:
        lines.append(f"| latency per IMU sample | processing p50 {f(la['processing_p50'], 3)} ms, p95 "
                     f"{f(la['processing_p95'], 3)} ms, max {f(la['processing_max'], 2)} ms; PX4 sample to output "
                     f"p50 {f(la['sample_to_output_p50'], 2)} ms, p95 {f(la['sample_to_output_p95'], 2)} ms"
                     + (f" ({la['clock_jump_samples']} samples left out: PX4's clock sync reset)"
                        if la.get("clock_jump_samples") else "")
                     + ("" if m.get("sim", {}).get("timing_reliable", True) else
                        "; **unreliable**: the simulation ran slower than real time, so this is PX4's clock drift")
                     + " |")
    sm = m.get("sim")
    if sm:
        lines.append(f"| simulation real-time factor | {f(sm['real_time_factor'], 3)} (slowest 30 s: "
                     f"{f(sm['rtf_min_30s'], 3)}). PX4-clocked timing is "
                     f"{'reliable' if sm['timing_reliable'] else 'unreliable'}: it needs every 30 s at "
                     f"{RTF_TIMING_MIN:g} or more |")
    return "\n".join(lines) + "\n"


def selfcheck():
    ok = True

    def ck(c, what):
        nonlocal ok
        print(f"{'PASS' if c else 'FAIL'}  selfcheck: {what}")
        ok &= bool(c)
    rng = np.random.default_rng(5)
    # A 2-D Gaussian error with the covariance it claims: NEES mean 2, 1.1% over 9, ~1.1% over 3 sigma(ish).
    P = np.array([[25.0, 6.0], [6.0, 16.0]])
    e = rng.multivariate_normal([0, 0], P, 200000)
    n = nees2(e, np.full(len(e), 25.0), np.full(len(e), 16.0), np.full(len(e), 6.0))
    ck(abs(n.mean() - 2) < 0.03 and abs(100 * (n > 9).mean() - 1.11) < 0.1, f"NEES of honest errors: mean {n.mean():.3f}, "
       f"over 9 {100 * (n > 9).mean():.2f}%")
    ck(np.isinf(nees2(np.array([[1.0, 1.0]]), np.array([0.0]), np.array([1.0]), np.array([0.0])))[0],
       "a covariance that isn't positive definite gives infinite NEES")
    # CEP of a Rayleigh error with sigma 10 per axis: CEP50 11.77, CEP95 24.48.
    r = np.hypot(*rng.normal(0, 10, (2, 400000)))
    ck(abs(cep(r, 50) - 11.77) < 0.1 and abs(cep(r, 95) - 24.48) < 0.2,
       f"CEP50 and CEP95 of a circular Gaussian: {cep(r, 50):.2f}, {cep(r, 95):.2f}")
    # Windows: a 1 Hz error ramp 0..399 m; the 5-minute window holds 0..300.
    t = np.arange(-10.0, 400.0)
    w = window_scores(t, np.maximum(t, 0), np.zeros(len(t), bool), 350.0, {300: (50.0, 100.0), 600: (75.0, 100.0)})
    ck(w["300"]["max_m"] == 300 and w["300"]["complete"] and not w["600"]["complete"] and w["600"]["max_m"] == 350 and
       not w["300"]["max_ok"], "score windows from the route's start, cut at the landing")
    # Time to acquire: 150 m falling to 20 m by t = 60, under 30 m from 55 s on.
    t = np.arange(0.0, 200.0, 0.5)
    err = np.maximum(150 - 2.3 * t, 20)
    tta = time_to_acquire(t, err, 30.0)
    ck(tta is not None and abs(tta - 52.5) < 0.6, f"time to acquire (under 30 m for 30 s): {tta}")
    ck(time_to_acquire(t, 150 + 0 * t, 30.0) is None, "never acquired: None")
    s = nis_stats(rng.chisquare(1, 100000), 1)
    ck(abs(s["mean"] - 1) < 0.02 and abs(s["over_997_pct"] - 0.3) < 0.06, f"NIS of honest 1-D innovations: mean "
       f"{s['mean']:.3f}, over the 99.7% point {s['over_997_pct']:.2f}%")
    # Integrity proxy: limit 30 m (flag when 3 sigma > 30, i.e. sigma > 10). The error ramps
    # 0 -> 200 m over 0..200 s and back to 0; sigma is 5 m until 120 s, then 20 m.
    t = np.arange(0.0, 400.0, 1.0)
    err = np.where(t <= 200, t, np.maximum(400 - t, 0))
    sig = np.where(t < 120, 5.0, 20.0)
    ig = integrity(t, err, sig, 30.0)
    ck(ig["judged"] == 1 and ig["detected"] == 1 and abs(ig["cases"][0]["t_s"] - 91) < 1e-9 and
       abs(ig["cases"][0]["latency_s"] - 29) < 1e-9, f"a lost lock flagged 29 s after the error passes 90 m: "
       f"{ig['cases']}")
    ig = integrity(t, err, np.where(t < 125, 5.0, 20.0), 30.0)
    ck(ig["judged"] == 1 and ig["detected"] == 0, "flagged 34 s after: not detected")
    ck(ig["false_alarms"] == 0, "a flag episode in which the error passes the limit isn't a false alarm")
    ig = integrity(t, np.where(t < 60, 5.0, 100.0), np.where(t > 30, 20.0, 5.0), 30.0)
    ck(ig["false_alarms"] == 0, "a warning that comes true (flagged at 5 m, then 100 m) isn't a false alarm")
    chat = (t > 10) & (t < 40) & (t % 4 < 2)
    ig = integrity(t, 0 * t + 5.0, np.where(chat, 20.0, 5.0), 30.0)
    ck(ig["episodes"] == 1 and ig["false_alarms"] == 1, "a flag chattering on and off is one episode")
    sig2 = np.where((t > 10) & (t < 20), 20.0, 5.0)
    ig = integrity(t, 0 * t + 5.0, sig2, 30.0)
    ck(ig["false_alarms"] == 1 and ig["judged"] == 0, "the flag coming on while the error is 5 m: one false alarm")
    tt = np.arange(0.0, 100.0)
    ig = integrity(tt, np.where(tt > 80, 100.0, 1.0), 0 * tt + 1.0, 30.0)
    ck(ig["judged"] == 0 and len(ig["cases"]) == 1, "a case the flight ends too soon after isn't judged")
    e2 = np.array([0, 100, 50, 100, 20, 100, 0.0])
    ig = integrity(np.arange(7.0) * 100, e2, 0 * e2 + 20.0, 30.0)
    ck(len(ig["cases"]) == 2, "a new case needs the error back under the limit first")
    print("SELFCHECK PASSED" if ok else "SELFCHECK FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("runs", nargs="*", help="tools/run_scenario.py run directories")
    ap.add_argument("--selfcheck", action="store_true")
    ap.add_argument("--no-plot", action="store_true", help="metrics.json and report.md only")
    a = ap.parse_args()
    if a.selfcheck:
        return selfcheck()
    if not a.runs:
        ap.error("give run directories or --selfcheck")
    ms = []
    for run in a.runs:
        d = read_run(run)
        m, series = score(d)
        (Path(run) / "metrics.json").write_text(json.dumps(m, indent=1) + "\n")
        save_series(series, Path(run) / "series.npz")
        if a.no_plot:
            (Path(run) / "report.md").write_text(report_md(m))
        else:
            report(m, series, d, run)
        w5 = m["windows"]["300"]
        print(f"{m['scenario']}: 5 min CEP95 {w5['cep95_m']:.1f} m (limit {w5['cep95_limit_m']:g}), max {w5['max_m']:.1f} m "
              f"(limit {w5['max_limit_m']:g}); over 3 sigma {m['consistency']['over3sigma_pct']:.1f}%; "
              f"report {Path(run) / 'report.md'}")
        ms.append(m)
    if len(ms) > 1:
        import score_set
        print(score_set.summary(score_set.pool(score_set.load(a.runs))))
    return True


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
