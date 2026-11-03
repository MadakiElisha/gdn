#!/usr/bin/env python3
"""Horizontal error of GDN (/gdn/odom) against Gazebo truth (/gdn/sim/truth_odom) in a bag.

    python3 tools/score_sitl_bag.py /tmp/gdn_sim/sitl_bag          # every 20 s
    python3 tools/score_sitl_bag.py BAG --step 10

Both are taken from the same bag, matched by receive time. Truth is moved to start at
GDN's origin (the take-off point). Prints the error at each step and RMS/max over the
first 5 minutes, first 10 minutes and the whole run.
"""
import argparse
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--step", type=float, default=20.0)
    a = ap.parse_args()
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    g, tr = [], []
    while r.has_next():
        topic, raw, t = r.read_next()
        if topic == "/gdn/odom":
            p = deserialize_message(raw, Odometry).pose.pose.position
            g.append((t * 1e-9, p.x, p.y))                  # NED north, east
        elif topic == "/gdn/sim/truth_odom":
            p = deserialize_message(raw, Odometry).pose.pose.position
            tr.append((t * 1e-9, p.y, p.x))                 # ENU -> north, east
    if not g or not tr:
        raise SystemExit("bag has no /gdn/odom or no /gdn/sim/truth_odom")
    g, tr = np.array(g), np.array(tr)
    t0 = g[0, 0]
    n0, e0 = np.interp(t0, tr[:, 0], tr[:, 1]), np.interp(t0, tr[:, 0], tr[:, 2])
    tn = np.interp(g[:, 0], tr[:, 0], tr[:, 1]) - n0
    te = np.interp(g[:, 0], tr[:, 0], tr[:, 2]) - e0
    err = np.hypot(g[:, 1] - tn, g[:, 2] - te)
    rel = g[:, 0] - t0
    print("t(s):err(m)  " + " ".join(f"{T:.0f}:{err[np.searchsorted(rel, T)]:.0f}"
                                      for T in np.arange(0, rel[-1], a.step)))
    for name, hi in (("0-300 s", 300), ("0-600 s", 600), ("whole run", 1e9)):
        e = err[rel <= hi]
        print(f"{name:>9}: RMS {np.sqrt(np.mean(e ** 2)):8.1f} m  MAX {e.max():8.1f} m")


if __name__ == "__main__":
    main()
