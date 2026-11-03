#!/usr/bin/env python3
"""Physics drop test: prove a generated Gazebo world matches GDN's terrain map.

Sets 1 m high-friction boxes down on the world's heightmap at known east/north
points (just above where the map says the ground is), lets physics settle them, then
compares where each one rests with the map height under it. A mirrored, shifted,
rotated or wrongly scaled world fails with errors of tens to hundreds of metres. A
correct one agrees to about a metre: Gazebo's collision surface is triangles, the map
is bilinear, and the image is oversampled to keep that gap small.

Needs gz-sim 8 (Harmonic) with Python bindings; no GPU and no PX4.

    python3 tools/make_sim_world.py                 # writes data/sim/zermatt/
    python3 tools/check_sim_world.py                # exit 0 = pass
"""
import argparse
import re
import sys
import tempfile
from pathlib import Path

import numpy as np

from gdn_paths import ROOT, load_map

HALF = 0.5          # half-size of the probe box, m


def read_info(path):
    info = {}
    for line in Path(path).read_text().splitlines():
        k, _, v = line.partition("=")
        info[k] = v
    return info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world-dir", default=str(ROOT / "data/sim/zermatt"))
    ap.add_argument("--points", type=int, default=30)
    ap.add_argument("--iterations", type=int, default=2500, help="physics steps (4 ms each)")
    ap.add_argument("--tol", type=float, default=1.0, help="pass if the median |rest - expected| <= tol (m)")
    a = ap.parse_args()

    import gz.math7  # noqa: F401  (needed so poses convert to Python)
    import gz.sim8 as gzs   # imported here so --help works without gz installed
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from make_sim_world import sample_map

    wdir = Path(a.world_dir)
    info = read_info(wdir / "world_info.txt")
    alt, hdr = load_map(info["map"])
    alt = alt.astype(np.float64)
    z0 = float(info["z0_amsl"])
    half_e, half_n = 0.40 * float(info["size_e"]), 0.40 * float(info["size_n"])

    def cos_slope(e, n, d=5.0):
        h = lambda ee, nn: float(sample_map(alt, hdr, np.array(ee), np.array(nn)))
        ge = (h(e + d, n) - h(e - d, n)) / (2 * d)
        gn = (h(e, n + d) - h(e, n - d)) / (2 * d)
        return 1.0 / np.sqrt(1.0 + ge * ge + gn * gn)

    rng = np.random.default_rng(1)
    pts = [(0.0, 0.0)] + [(rng.uniform(-half_e, half_e), rng.uniform(-half_n, half_n))
                          for _ in range(a.points - 1)]

    # Test world = the generated world with the drone include swapped for spheres.
    sdf = (wdir / f"{info['world']}.sdf").read_text()
    sdf = re.sub(r"\s*<include>.*?</include>", "", sdf, flags=re.S)
    spheres = []
    for k, (e, n) in enumerate(pts):
        z = float(sample_map(alt, hdr, np.array(e), np.array(n))) - z0 + HALF / cos_slope(e, n) + 0.3
        spheres.append(f"""
    <model name="probe_{k}">
      <pose>{e:.3f} {n:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>
        <collision name="c"><geometry><box><size>{2*HALF} {2*HALF} {2*HALF}</size></box></geometry>
          <surface><friction><ode><mu>20</mu><mu2>20</mu2></ode></friction></surface></collision>
      </link>
    </model>""")
    sdf = sdf.replace("</world>", "".join(spheres) + "\n  </world>")
    tmp = Path(tempfile.mkdtemp()) / "drop_test.sdf"
    tmp.write_text(sdf)

    rest, before = {}, {}
    check_at = a.iterations - 250          # 1 s earlier: used to tell settled from rolling

    def on_post_update(info_, ecm):
        if info_.paused or info_.iterations not in (check_at, a.iterations):
            return
        store = before if info_.iterations == check_at else rest
        world = gzs.World(gzs.world_entity(ecm))
        for k in range(len(pts)):
            m = gzs.Model(world.model_by_name(ecm, f"probe_{k}"))
            link = gzs.Link(m.canonical_link(ecm))
            p = link.world_pose(ecm)
            if p is not None:
                pos = p.pos()
                store[k] = (pos.x(), pos.y(), pos.z())

    fx = gzs.TestFixture(str(tmp))
    fx.on_post_update(on_post_update)
    fx.finalize()
    fx.server().run(True, a.iterations, False)

    if len(rest) != len(pts):
        sys.exit(f"FAIL: only {len(rest)}/{len(pts)} probe poses read back")
    errs = []
    print(f"{'probe':>5} {'east':>9} {'north':>9} {'rest z':>9} {'map z':>9} {'diff':>7}  state")
    for k in range(len(pts)):
        x, y, z = rest[k]
        moved = np.linalg.norm(np.subtract(rest[k], before.get(k, rest[k])))
        settled = moved < 0.2                       # less than 0.2 m in the last second
        expect = float(sample_map(alt, hdr, np.array(x), np.array(y))) - z0 + HALF / cos_slope(x, y)
        d = z - expect
        if settled:
            errs.append(d)
        print(f"{k:5d} {x:9.1f} {y:9.1f} {z:9.2f} {expect:9.2f} {d:7.2f}  "
              f"{'settled' if settled else f'still moving ({moved:.1f} m/s)'}")
    if len(errs) < 0.6 * len(pts):
        sys.exit(f"FAIL: only {len(errs)}/{len(pts)} probes settled; run more --iterations")
    errs = np.abs(np.array(errs))
    med = float(np.median(errs))

    # Negative controls: the same rest points scored against a mirrored or shifted
    # map must be far worse, otherwise this test couldn't catch those mistakes.
    def med_if(fx, fy, dx=0.0, dy=0.0):
        d = []
        for k in range(len(pts)):
            x, y, z = rest[k]
            h = float(sample_map(alt, hdr, np.array(fx * x + dx), np.array(fy * y + dy))) - z0
            d.append(abs(z - (h + HALF)))
        return float(np.median(d))
    controls = {"mirrored east-west": med_if(-1, 1), "mirrored north-south": med_if(1, -1),
                "shifted 50 m east": med_if(1, 1, 50.0), "shifted 50 m north": med_if(1, 1, 0.0, 50.0)}
    pad_err = abs(errs[0]) if 0 in rest else 99.0

    print(f"settled probes: {len(errs)}/{len(pts)};  pad error {pad_err:.2f} m;  "
          f"median {med:.2f} m;  p90 {np.percentile(errs, 90):.2f} m;  max {errs.max():.2f} m")
    for name, m in controls.items():
        print(f"  control, {name:22s}: median {m:7.1f} m")
    checks = {
        "pad within 0.2 m": pad_err <= 0.2,
        f"median within {a.tol} m": med <= a.tol,
        "every control at least 10x worse": all(m >= 10 * max(med, 0.1) for m in controls.values()),
    }
    for name, ok in checks.items():
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    print("note: Gazebo's physics surface is triangulated, so single probes on very steep cells can be")
    print("      several metres off. The altimeter in SITL measures the rendered terrain instead, and")
    print("      sim_adapter.py reports its agreement with the map live during a run.")
    sys.exit(0 if all(checks.values()) else 1)


if __name__ == "__main__":
    main()
