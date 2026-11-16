#!/usr/bin/env python3
"""Did PX4 really lose GPS in a Stage B run (GPS_OFF_AT)?

    python3 tools/check_gps_cut.py log/gdn_runs/gdn_<time>              # expects the cut
    python3 tools/check_gps_cut.py log/gdn_runs/gdn_<time> --expect kept # a Stage A run

Reads PX4's own position estimate (/fmu/out/vehicle_local_position) from GDN's run log
(R1.11) and looks at the airborne part: from the first sample more than 2 m above the
start to the last. PX4 "lost GPS" if its horizontal position went invalid (xy_valid
false) or its horizontal uncertainty (eph) passed --eph-lost metres. With GPS on, the
x500 in SITL holds eph under 1 m.

Exit 0 if the run did what --expect says, 1 if not, 2 if the log can't be read.
"""
import argparse
import sys
from pathlib import Path


def read_lpos(run_log):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(run_log), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    out = []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == "/fmu/out/vehicle_local_position":
            m = deserialize_message(data, types[topic])
            out.append((m.timestamp * 1e-6, m.z, bool(m.z_valid), bool(m.xy_valid), float(m.eph)))
    return out


def judge(lpos, eph_lost):
    """Returns (lost, text) for the airborne part of lpos [(t, z, z_valid, xy_valid, eph)]."""
    if not lpos:
        return None, "no /fmu/out/vehicle_local_position in the run log"
    t0 = lpos[0][0]
    z0 = next((z for _, z, zv, _, _ in lpos if zv), None)
    up = [i for i, (_, z, zv, _, _) in enumerate(lpos) if zv and z0 is not None and z < z0 - 2.0]
    if not up:
        return None, "PX4 never climbed 2 m above the start"
    air = lpos[up[0]:up[-1] + 1]
    eph_max = max(e for *_, e in air)
    invalid = [t for t, _, _, xv, _ in air if not xv]
    big = [t for t, *_, e in air if e > eph_lost]
    first = min(invalid[:1] + big[:1]) if (invalid or big) else None
    text = (f"airborne t+{air[0][0] - t0:.0f} to t+{air[-1][0] - t0:.0f} s: PX4 eph max {eph_max:.1f} m, "
            f"horizontal position invalid in {len(invalid)} of {len(air)} samples")
    if first is not None:
        text += f"; GPS lost from t+{first - t0:.0f} s"
    return first is not None, text


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("run_log", help="a GDN run log directory (log/gdn_runs/gdn_<time>)")
    ap.add_argument("--expect", choices=("lost", "kept"), default="lost")
    ap.add_argument("--eph-lost", type=float, default=10.0, dest="eph_lost",
                    help="PX4 eph above this (m) counts as GPS lost")
    a = ap.parse_args()
    if not any(Path(a.run_log).glob("*.mcap")):
        print(f"check_gps_cut: no .mcap in {a.run_log}")
        return 2
    lost, text = judge(read_lpos(a.run_log), a.eph_lost)
    if lost is None:
        print(f"FAIL  {text}")
        return 2
    ok = lost == (a.expect == "lost")
    print(f"{'PASS' if ok else 'FAIL'}  GPS {a.expect} expected; {text}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
