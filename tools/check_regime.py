#!/usr/bin/env python3
"""Flight regime check (roadmap R2.5b, TEST-039): does GDN follow PX4's regime and its rules?

    python3 tools/check_regime.py --run LOG            # a tools/sim_run.sh log directory
    python3 tools/check_regime.py --log RUN_LOG --vehicle x500   # a GDN run log (log/gdn_runs/...)
    python3 tools/check_regime.py --log REPLAY --vehicle x500 --px4-from RUN_LOG
                                   # a gdn_replay --no-regime log, judged against the flight's
                                   # own PX4 regime (from the original run log)
    python3 tools/check_regime.py --selfcheck          # the regime table and the interval logic

Reads GDN's run log (R1.11): its inputs (PX4's vehicle_land_detected and vehicle_status),
its events (regime changes), its updates and its states. Needs ROS 2 (the dev image).

  - PX4's land detector and status reached GDN (they're in its run log);
  - GDN's regime follows PX4's: the regimes this kind of vehicle flies (multirotor:
    ground, hover; VTOL: and transition, cruise; fixed-wing: ground, cruise), each change
    within 0.5 s of PX4's;
  - zero-velocity updates only on the ground: none applied while PX4 says airborne;
  - re-armed after landing: ZUPT applied again once landed, and GDN's speed over the last
    5 s on the ground under 0.3 m/s (the true speed is zero);
  - the terrain altimeter isn't applied on the ground (the profiles' rule).
"""
import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REGIMES = ("unknown", "ground", "hover", "transition", "cruise")
EXPECT = {"multirotor": {"ground", "hover"}, "vtol": {"ground", "hover", "transition", "cruise"},
          "fixedwing": {"ground", "cruise"}}
S_AGL, S_ZUPT = 2, 6
APPLIED = 0
ARMED, ROTARY, FIXED = 2, 1, 2
SPEED_MAX, SETTLE_S, LAG_MAX_S = 0.3, 5.0, 0.5


def ck(ok, what, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {what}{': ' + detail if detail else ''}")
    return bool(ok)


def px4_regime(armed, vehicle_type, in_transition, landed):
    """The regime from PX4's status and land detector, as gdn::RegimeFromPx4."""
    if landed or armed != ARMED:
        return "ground"
    if in_transition:
        return "transition"
    return "cruise" if vehicle_type == FIXED else "hover"


def px4_track(status, land):
    """[(t, regime)] whenever PX4's regime changes. status: [(t, armed, type, transition)],
    land: [(t, landed)], both sorted; a regime exists once both have arrived."""
    ev = sorted([(t, "s", r) for t, *r in status] + [(t, "l", (l,)) for t, l in land])
    st, ld, out = None, None, []
    for t, kind, r in ev:
        if kind == "s":
            st = r
        else:
            ld = r[0]
        if st is None or ld is None:
            continue
        g = px4_regime(st[0], st[1], st[2], ld)
        if not out or out[-1][1] != g:
            out.append((t, g))
    return out


def regime_at(track, t):
    """The regime at times t from a change track [(t, regime)] ('unknown' before the first)."""
    ts = np.array([x[0] for x in track]) if track else np.zeros(0)
    k = np.searchsorted(ts, t, side="right") - 1
    return np.array(["unknown" if i < 0 else track[i][1] for i in np.atleast_1d(k)])


def read_log(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    d = {"status": [], "land": [], "events": [], "updates": [], "states": [], "profile": None}
    last = -1.0
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic not in types:
            continue
        if topic == "/fmu/out/vehicle_status_v1":
            m = deserialize_message(raw, types[topic])
            d["status"].append((m.timestamp * 1e-6, m.arming_state, m.vehicle_type, bool(m.in_transition_mode)))
        elif topic == "/fmu/out/vehicle_land_detected":
            m = deserialize_message(raw, types[topic])
            d["land"].append((m.timestamp * 1e-6, bool(m.landed)))
        elif topic == "/gdn/log/event":
            m = deserialize_message(raw, types[topic])
            d["events"].append((m.t, m.text))
        elif topic == "/gdn/log/update":
            m = deserialize_message(raw, types[topic])
            d["updates"].append((m.t, m.sensor, m.reason))
        elif topic == "/gdn/log/state":
            m = deserialize_message(raw, types[topic])
            if m.flags & 1 and m.t > 0 and m.t >= last + 0.05:
                last = m.t
                d["states"].append((m.t, *m.velocity, m.flags))
        elif topic == "/gdn/log/vehicle_profile":
            d["profile"] = deserialize_message(raw, types[topic]).name
    for k in ("status", "land", "updates", "states"):
        d[k] = sorted(d[k])
    return d


def gdn_track(events):
    """[(t, regime)] from GDN's "regime: A -> B" events."""
    out = []
    for t, text in events:
        m = re.search(r"regime: (\w+) -> (\w+)", text)
        if m:
            out.append((t, m.group(2)))
    return out


def check(d, kind, label):
    ok = True
    ok &= ck(len(d["land"]) > 0 and len(d["status"]) > 0, f"{label}: PX4's land detector and status available",
             f"{len(d['land'])} land-detector, {len(d['status'])} status messages in its run log")
    if not (d["land"] and d["status"]):
        return False
    px4 = px4_track(d["status"], d["land"])
    gdn = gdn_track(d["events"])
    seen = {g for _, g in gdn}
    ok &= ck(EXPECT[kind] <= seen, f"{label}: GDN went through a {kind}'s regimes ({', '.join(sorted(EXPECT[kind]))})",
             " -> ".join(g for _, g in gdn) or "no regime events")
    # Each PX4 change matched by GDN's within LAG_MAX_S.
    # GDN's clock reads 0 until static alignment, so a change it logged before then (the
    # first "ground") is stamped t=0: it matches PX4's first regime.
    lags = []
    for k, (t, g) in enumerate(px4):
        if k == 0 and gdn and gdn[0][0] <= 0 and gdn[0][1] == g:
            lags.append(0.0)
            continue
        later = [tg for tg, gg in gdn if gg == g and tg >= t - 0.05]
        lags.append(later[0] - t if later else math.inf)
    worst = max(lags) if lags else math.inf
    ok &= ck(worst <= LAG_MAX_S, f"{label}: each PX4 regime change followed by GDN within {LAG_MAX_S} s",
             f"{len(px4)} changes, worst {worst:.3f} s")
    up = np.array(d["updates"], dtype=float) if d["updates"] else np.zeros((0, 3))
    zapp = up[(up[:, 1] == S_ZUPT) & (up[:, 2] == APPLIED), 0] if len(up) else np.zeros(0)
    # An update at exactly the time of PX4's change (the IMU sample PX4 stamped its message
    # with, processed just before the message) belongs to the regime before it.
    reg_z = regime_at(px4, zapp - 1e-6)
    air = np.isin(reg_z, ["hover", "transition", "cruise"])
    ok &= ck(not air.any(), f"{label}: no zero-velocity update applied while PX4 says airborne",
             f"{int(air.sum())} of {len(zapp)} applied ZUPTs in the air")
    # After the last landing.
    land_t = [t for t, g in px4 if g == "ground" and any(gg != "ground" for tt, gg in px4 if tt < t)]
    if ck(len(land_t) > 0, f"{label}: PX4 landed after flying", f"at {land_t[-1] - px4[0][0]:.1f} s" if land_t else "never"):
        tl = land_t[-1]
        after = zapp[zapp > tl]
        ok &= ck(len(after) > 0, f"{label}: zero-velocity updates re-armed after landing",
                 f"{len(after)} applied after landing")
        st = np.array(d["states"], dtype=float)
        tail = st[st[:, 0] >= st[-1, 0] - SETTLE_S] if len(st) else st
        tail = tail[tail[:, 0] > tl + 1.0] if len(tail) else tail
        sp = np.hypot(tail[:, 1], tail[:, 2]) if len(tail) else np.array([math.inf])
        ok &= ck(len(tail) > 0 and np.median(sp) < SPEED_MAX,
                 f"{label}: GDN's speed on the ground after landing under {SPEED_MAX} m/s",
                 f"median {np.median(sp):.3f} m/s over the last {SETTLE_S:.0f} s")
    else:
        ok = False
    aapp = up[(up[:, 1] == S_AGL) & (up[:, 2] == APPLIED), 0] if len(up) else np.zeros(0)
    reg_a = regime_at(px4, aapp - 1e-6)
    n_ground = int((reg_a == "ground").sum())
    ok &= ck(n_ground == 0, f"{label}: terrain altimeter not applied on the ground", f"{n_ground} of {len(aapp)} applied")
    return ok


def run_log_of(run_dir):
    g = Path(run_dir) / "gdn.log"
    m = re.search(r"run log: (\S+)", g.read_text()) if g.exists() else None
    if not m:
        sys.exit(f"no 'run log:' line in {g}")
    return Path(m.group(1))


def main_run(run_dir, log, vehicle, px4_from=None):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import sim_vehicles
    if run_dir:
        info = dict(l.split("=", 1) for l in (Path(run_dir) / "run_info.txt").read_text().splitlines() if "=" in l)
        vehicle, log = info["vehicle"], run_log_of(run_dir)
    kind = sim_vehicles.get(vehicle).kind
    d = read_log(log)
    label = vehicle
    if px4_from:                        # PX4's regime from the flight, GDN's behaviour from the replay
        src = read_log(px4_from)
        d["status"], d["land"] = src["status"], src["land"]
        label = f"{vehicle} (replay; PX4's regime from the flight's log)"
    ok = check(d, kind, label)
    print("REGIME CHECK PASSED" if ok else "REGIME CHECK FAILED")
    return ok


def selfcheck():
    ok = True
    table = [((2, ROTARY, False, False), "hover"), ((2, ROTARY, False, True), "ground"), ((1, ROTARY, False, False), "ground"),
             ((2, FIXED, False, False), "cruise"), ((2, ROTARY, True, False), "transition"),
             ((2, FIXED, True, False), "transition"), ((1, FIXED, False, False), "ground"), ((2, FIXED, False, True), "ground")]
    ok &= ck(all(px4_regime(*a) == r for a, r in table), "selfcheck: the regime table (armed, type, transition, landed)")
    tr = px4_track([(0.0, 1, ROTARY, False), (5.0, 2, ROTARY, False), (60.0, 2, ROTARY, False)],
                   [(0.5, True), (10.0, False), (50.0, True)])
    ok &= ck([g for _, g in tr] == ["ground", "hover", "ground"] and tr[1][0] == 10.0 and tr[2][0] == 50.0,
             "selfcheck: a take-off and a landing from status and land detector", str(tr))
    ok &= ck(list(regime_at(tr, np.array([0.0, 0.6, 20.0, 55.0]))) == ["unknown", "ground", "hover", "ground"],
             "selfcheck: regime at given times")
    ok &= ck(gdn_track([(1.0, "regime: unknown -> ground (PX4: landed)"), (3.0, "other"),
                        (9.0, "regime: ground -> hover (PX4: armed, rotary wing, airborne)")]) ==
             [(1.0, "ground"), (9.0, "hover")], "selfcheck: GDN's regime events read")
    vt = px4_track([(0.0, 2, ROTARY, False), (20.0, 2, ROTARY, True), (24.0, 2, FIXED, False)], [(0.0, False)])
    ok &= ck([g for _, g in vt] == ["hover", "transition", "cruise"], "selfcheck: a VTOL's front transition")
    ok &= ck(list(regime_at(tr, np.array([10.0 - 1e-6, 10.0]))) == ["ground", "hover"],
             "selfcheck: an update at the instant of PX4's change belongs to the regime before (t - 1e-6)")
    # A whole made-up run: GDN's first change logged before alignment (t=0), one ZUPT tied
    # with the take-off, ZUPT re-armed after landing, speed zero at the end.
    d = {"status": [(0.0, 1, ROTARY, False), (5.0, 2, ROTARY, False)], "land": [(0.5, True), (10.0, False), (50.0, True)],
         "events": [(0.0, "regime: unknown -> ground"), (10.0, "regime: ground -> hover"), (50.0, "regime: hover -> ground")],
         "updates": [(9.0, S_ZUPT, APPLIED), (10.0, S_ZUPT, APPLIED), (52.0, S_ZUPT, APPLIED), (20.0, S_AGL, APPLIED)],
         "states": [(t, 0.0, 0.0, 0.0, 0) for t in np.arange(51.0, 60.0, 0.5)]}
    ok &= ck(check(d, "multirotor", "selfcheck run"), "selfcheck: a good made-up run passes the whole check")
    d["updates"].append((30.0, S_ZUPT, APPLIED))
    ok &= ck(not check(d, "multirotor", "selfcheck run, a ZUPT in flight"), "selfcheck: a ZUPT in flight fails it")
    print("SELFCHECK PASSED" if ok else "SELFCHECK FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", help="a tools/sim_run.sh log directory")
    ap.add_argument("--log", help="a GDN run log directory (with --vehicle)")
    ap.add_argument("--vehicle", default="x500")
    ap.add_argument("--px4-from", help="with --log: take PX4's status and land detector from this run log")
    ap.add_argument("--selfcheck", action="store_true")
    a = ap.parse_args()
    if a.selfcheck:
        return selfcheck()
    if not (a.run or a.log):
        ap.error("give --run, --log or --selfcheck")
    return main_run(a.run, a.log, a.vehicle, a.px4_from)


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
