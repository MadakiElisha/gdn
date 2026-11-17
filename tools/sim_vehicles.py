#!/usr/bin/env python3
"""The simulated vehicles (roadmap R2.3): which Gazebo model, PX4 airframe and settings
each uses, and how tools/sim_fly.py flies it.

    python3 tools/sim_vehicles.py --list
    eval "$(python3 tools/sim_vehicles.py shell standard_vtol)"      # VEHICLE_* variables
    python3 tools/sim_vehicles.py world rc_cessna data/sim/zermatt out.sdf [runway|catapult]

Every vehicle is PX4's own Gazebo model with GDN's payload (sim/models/gdn_payload)
included and fixed to its base_link: sim/models/<model>/model.sdf.

  x500           quadrotor. Offboard position setpoints along the route, lands on the pad.
  standard_vtol  quadplane. Climbs like the x500, changes to fixed-wing flight towards
                 the route, flies it as a plane, changes back near the pad and lands.
  rc_cessna      fixed-wing. Takes off along the world's runway (make_sim_world.py's
                 graded strip through the pad) with PX4's runway take-off, flies the
                 route as a plane, and lands back on the runway (a landing mission).
                 Launch "catapult" instead (GDN_FW_LAUNCH=catapult): Gazebo pushes it
                 for about half a second (ApplyLinkWrench), PX4's launch detection sees
                 the push and it climbs out; it still lands on the runway.

The worlds (tools/make_sim_world.py) place the x500. `world` writes a copy for a run
with the chosen vehicle in its place, the fixed-wing facing down the runway (or its
catapult heading).
"""
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Vehicle:
    model: str             # sim/models/<model>
    airframe: str          # PX4 ROMFS/px4fmu_common/init.d-posix/airframes/<airframe>
    kind: str              # multirotor | vtol | fixedwing
    speed: float           # m/s along the route (multirotor carrot; fixed-wing only for timing)
    climb_grad: float = 0.0   # steepest climb or descent the route may ask for (0: any)
    params: dict = field(default_factory=dict)   # PX4 parameters on top of COMMON_PARAMS

    @property
    def autostart(self):
        return int(self.airframe.split("_")[0])

    @property
    def instance(self):
        return f"{self.model}_0"


# PX4 parameters for every vehicle, set through px4-param after PX4 starts:
# - IMU_INTEG_RATE 100: uXRCE-DDS forwards sensor_combined at most at 100 Hz, while PX4
#   integrates the IMU at 250 Hz by default. Then each message GDN gets covers only
#   4 of every 10 ms and 60% of the motion is lost, which wrecks the attitude on a
#   multirotor. At 100 Hz every message covers its whole interval.
# - NAV_DLL_ACT 0: no fail-safe for the missing ground station (QGroundControl).
# - EKF2_GPS_CTRL 7 (PX4's default): GPS on. A Stage B cut sets it to 0 mid-flight
#   (sim_fly.py --gps-off-at); PX4 keeps parameters between runs, so set it back here.
COMMON_PARAMS = {"IMU_INTEG_RATE": 100, "NAV_DLL_ACT": 0, "EKF2_GPS_CTRL": 7}

# Fixed-wing launch, by GDN_FW_LAUNCH:
# - runway: RWTO_TKOFF 1 (the airframe's default): Takeoff mode rolls down the runway
#   it faces, rotates and climbs out. No launch detection.
# - catapult (sim_fly.py pushes, PX4 detects): RWTO_TKOFF 0; FW_LAUN_DETCN_ON 1, so PX4
#   waits in Takeoff mode, motor at idle, until the body forward acceleration passes
#   FW_LAUN_AC_THLD (30 m/s2) for FW_LAUN_AC_T (0.05 s).
# - both: MIS_TAKEOFF_ALT, the climb-out height above the pad before the route; and
#   FW_LND_USETER 0 for the landing on the runway: the rc_cessna has no rangefinder, so
#   with PX4's default (1) it found no terrain measurement on final approach and aborted
#   (FW_LND_ABORT); with 0 it takes the touchdown point's altitude as the ground, which
#   on the graded runway is exact.
LAUNCH_PARAMS = {"runway": {"RWTO_TKOFF": 1, "FW_LAUN_DETCN_ON": 0, "MIS_TAKEOFF_ALT": 50, "FW_LND_USETER": 0},
                 "catapult": {"RWTO_TKOFF": 0, "FW_LAUN_DETCN_ON": 1, "MIS_TAKEOFF_ALT": 50, "FW_LND_USETER": 0}}

VEHICLES = {
    "x500": Vehicle("x500_gdn", "4001_gz_x500", "multirotor", speed=10.0),
    # Fixed-wing climb limits: FW_T_CLMB_MAX 8 (VTOL) and 5 m/s (cessna) at about
    # 15 m/s airspeed; the route asks for at most half of that.
    # VT_F_TRANS_THR 0.6: the pusher during the front transition. With the airframe's 0.3
    # the VTOL reached only 7.7 to 8 m/s when PX4 checks it (VT_ARSP_BLEND 8 m/s after
    # VT_F_TR_OL_TM 6 s, scaled for air density), and one of five transitions at
    # Zermatt's 2600 m was aborted (quad-chute; TEST-034).
    "standard_vtol": Vehicle("standard_vtol_gdn", "4004_gz_standard_vtol", "vtol", speed=15.0, climb_grad=0.25,
                             params={"VT_F_TRANS_THR": 0.6}),
    "rc_cessna": Vehicle("rc_cessna_gdn", "4003_gz_rc_cessna", "fixedwing", speed=15.0, climb_grad=0.17),
}


def get(name):
    if name not in VEHICLES:
        sys.exit(f"sim_vehicles: no vehicle {name!r} (vehicles: {', '.join(VEHICLES)})")
    return VEHICLES[name]


def runway(info):
    """The world's runway from its world_info dict: heading (deg), grade, along-track
    span (from, to; m from the pad, positive along the heading) and width; or None."""
    if "runway_heading" not in info:
        return None
    return {k: float(info[f"runway_{k}"]) for k in ("heading", "grade", "from", "to", "width")}


def params(v, launch):
    out = dict(COMMON_PARAMS, **v.params)
    if v.kind == "fixedwing":
        out.update(LAUNCH_PARAMS[launch])
    return out


def launch_heading(alt, hdr, grad=0.12, dist=1500.0, step=20.0):
    """Heading (deg from north, clockwise) to launch a fixed-wing from the pad: the one
    whose terrain stays furthest below a line climbing at `grad` from 10 m above the
    pad, over `dist` metres. Returns (heading, worst margin in m)."""
    import numpy as np
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from make_sim_world import sample_map
    base = float(sample_map(alt, hdr, np.array(0.0), np.array(0.0)))
    d = np.arange(step, dist + step / 2, step)
    best = None
    for h in range(0, 360, 5):
        n, e = d * math.cos(math.radians(h)), d * math.sin(math.radians(h))
        margin = float(np.min(10.0 + grad * d - (sample_map(alt, hdr, e, n) - base)))
        if best is None or margin > best[1]:
            best = (float(h), margin)
    return best


def write_world(name, wdir, out, launch="runway"):
    """Copy wdir's world with vehicle `name` in place of the x500: the fixed-wing facing
    down the runway (or its catapult launch heading), the others facing east as before
    (yaw 0 in Gazebo's ENU)."""
    v = get(name)
    wdir = Path(wdir)
    info = dict(l.split("=", 1) for l in (wdir / "world_info.txt").read_text().split("\n") if "=" in l)
    sdf = (wdir / f"{info['world']}.sdf").read_text()
    inc = re.search(r"<include>\s*<uri>model://[^<]+</uri>\s*<name>[^<]+</name>\s*<pose>([^<]+)</pose>\s*</include>", sdf)
    if inc is None:
        sys.exit(f"sim_vehicles: no vehicle <include> in {wdir}/{info['world']}.sdf")
    x, y, z = inc.group(1).split()[:3]
    yaw = 0.0
    note = ""
    rw = runway(info)
    if v.kind == "fixedwing" and launch == "runway":
        if rw is None:
            sys.exit(f"sim_vehicles: {wdir} has no runway (rebuild it, or GDN_FW_LAUNCH=catapult)")
        yaw = math.radians(90.0 - rw["heading"])
        note = (f"; on the runway, heading {rw['heading']:.0f} deg, {rw['to']:.0f} m of it ahead "
                f"(grade {100 * rw['grade']:+.1f}%)")
    elif v.kind == "fixedwing":
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from gdn_paths import load_map
        alt, hdr = load_map(info.get("world_map", info["map"]))
        hdg, margin = launch_heading(alt.astype("float64"), hdr)
        yaw = math.radians(90.0 - hdg)       # ENU yaw from a compass heading
        note = f"; launch heading {hdg:.0f} deg (terrain at least {margin:.0f} m under a 12% climb)"
    block = (f"<include>\n      <uri>model://{v.model}</uri>\n      <name>{v.instance}</name>\n"
             f"      <pose>{x} {y} {z} 0 0 {yaw:.4f}</pose>\n    </include>")
    Path(out).write_text(sdf[:inc.start()] + block + sdf[inc.end():])
    print(f"sim_vehicles: {out}: {name} ({v.model}) on the pad{note}")


def main(argv):
    if not argv or argv[0] in ("-h", "--help"):
        sys.exit(__doc__)
    if argv[0] == "--list":
        for n, v in VEHICLES.items():
            print(f"{n:14s} {v.kind:10s} model {v.model}, PX4 airframe {v.airframe}")
    elif argv[0] == "shell" and len(argv) in (2, 3):
        v = get(argv[1])
        launch = argv[2] if len(argv) == 3 else "runway"
        if launch not in LAUNCH_PARAMS:
            sys.exit(f"sim_vehicles: launch {launch!r} is runway or catapult")
        ps = " ".join(f"{k}={val}" for k, val in params(v, launch).items())
        print(f"VEHICLE_MODEL={v.model}\nVEHICLE_INSTANCE={v.instance}\nVEHICLE_AUTOSTART={v.autostart}\n"
              f"VEHICLE_KIND={v.kind}\nVEHICLE_PARAMS='{ps}'")
    elif argv[0] == "world" and len(argv) in (4, 5):
        write_world(*argv[1:])
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main(sys.argv[1:])
