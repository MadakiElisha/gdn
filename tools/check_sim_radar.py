#!/usr/bin/env python3
"""Check that the rendered radar altimeter agrees with GDN's map.

Starts a headless Gazebo server with the generated world, a static probe that carries
the x500_gdn radar altimeter, and only the systems needed to render it. Puts the probe
100 m above the map at random points and compares the measured range with 100 m.
This is what the filter relies on in SITL, so run it after changing the world, the
model or the Gazebo version.

    python3 tools/check_sim_radar.py                     # Ogre2, exit 0 = pass
    python3 tools/check_sim_radar.py --render ogre       # Ogre1

Needs Gazebo Harmonic with Python bindings (python3-gz-transport13) and a GPU, or a
virtual display (xvfb-run python3 tools/check_sim_radar.py).
"""
import argparse
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

from gdn_paths import ROOT, load_map
from make_sim_world import sample_map


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world-dir", default=str(ROOT / "data/sim/zermatt"))
    ap.add_argument("--render", default="ogre2", choices=("ogre2", "ogre"))
    ap.add_argument("--points", type=int, default=25)
    ap.add_argument("--tol", type=float, default=2.5, help="fail if any |error| > tol (m)")
    ap.add_argument("--tol-median", type=float, default=0.5, help="fail if |median error| > this (m)")
    a = ap.parse_args()
    from gz.transport13 import Node
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.boolean_pb2 import Boolean

    wdir = Path(a.world_dir)
    info = dict(l.split("=", 1) for l in (wdir / "world_info.txt").read_text().split() if "=" in l)
    alt, hdr = load_map(info["map"])
    alt = alt.astype(np.float64)
    z0 = float(info["z0_amsl"])
    tmp = Path(tempfile.mkdtemp())
    sdf = re.sub(r"\s*<include>.*?</include>", "", (wdir / f"{info['world']}.sdf").read_text(), flags=re.S)
    model = (ROOT / "sim/models/x500_gdn/model.sdf").read_text()
    sensor = re.search(r'<sensor name="gdn_agl_radar".*?</sensor>', model, re.S).group(0)
    sdf = sdf.replace("</world>", f'<model name="probe"><static>true</static><pose>0 0 {float(info["pad_z"]) + 100}'
                      f' 0 0 0</pose><link name="base_link">{sensor}</link></model>\n  </world>')
    (tmp / "radar_check.sdf").write_text(sdf)
    plugins = "".join(f'<plugin entity_name="*" entity_type="world" filename="gz-sim-{f}-system" name="gz::sim::systems::{n}"/>'
                      for f, n in (("physics", "Physics"), ("user-commands", "UserCommands"),
                                   ("scene-broadcaster", "SceneBroadcaster")))
    plugins += (f'<plugin entity_name="*" entity_type="world" filename="gz-sim-sensors-system" '
                f'name="gz::sim::systems::Sensors"><render_engine>{a.render}</render_engine></plugin>')
    (tmp / "server.config").write_text(f"<server_config><plugins>{plugins}</plugins></server_config>")
    env = dict(os.environ, GZ_SIM_SERVER_CONFIG_PATH=str(tmp / "server.config"), GZ_IP="127.0.0.1")
    log = open(tmp / "gz.log", "w")
    srv = subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "3", str(tmp / "radar_check.sdf")],
                           env=env, stdout=log, stderr=subprocess.STDOUT)
    os.environ["GZ_IP"] = "127.0.0.1"
    try:
        node = Node()
        last = {}
        node.subscribe(LaserScan, "/gdn/agl_radar",
                       lambda m: last.update(r=(m.ranges[0] if len(m.ranges) else float("nan")), t=time.time()))
        t_start = time.time()
        while "t" not in last:
            if srv.poll() is not None or time.time() - t_start > 120:
                sys.exit(f"FAIL: no radar data (Gazebo log: {tmp / 'gz.log'})")
            time.sleep(0.2)
        rng = np.random.default_rng(3)
        half_e, half_n = 0.38 * float(info["size_e"]), 0.38 * float(info["size_n"])
        pts = [(0.0, 0.0)] + [(rng.uniform(-half_e, half_e), rng.uniform(-half_n, half_n)) for _ in range(a.points)]
        errs = []
        for e, n in pts:
            p = Pose()
            p.name = "probe"
            p.position.x, p.position.y = e, n
            p.position.z = float(sample_map(alt, hdr, np.array(e), np.array(n))) - z0 + 100.0
            node.request(f"/world/{info['world']}/set_pose", p, Pose, Boolean, 2000)
            time.sleep(0.5)
            t0 = time.time()
            while last["t"] < t0 + 0.2:
                time.sleep(0.05)
            errs.append(last["r"] - 100.0)
            print(f"  east {e:7.0f}  north {n:7.0f}  range {last['r']:8.2f} m  error {errs[-1]:+7.2f} m")
        d = np.array(errs)
        bad = ~np.isfinite(d) | (np.abs(d) > a.tol)
        print(f"{a.render}: {np.sum(~bad)}/{len(d)} within {a.tol} m; median {np.nanmedian(d):+.2f} m, "
              f"max |error| {np.nanmax(np.abs(d)):.2f} m")
        ok = not bad.any() and abs(np.nanmedian(d)) <= a.tol_median
        print("PASS" if ok else "FAIL: the rendered terrain doesn't match the map")
        sys.exit(0 if ok else 1)
    finally:
        srv.terminate()
        try:
            srv.wait(10)
        except subprocess.TimeoutExpired:
            srv.kill()


if __name__ == "__main__":
    main()
