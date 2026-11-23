#!/usr/bin/env python3
"""A steady wind for one SITL run (roadmap R2.6a; tools/sim_run.sh calls it for GDN_WIND).

    python3 tools/sim_wind.py --world RUN.sdf --server-config LOG/server.config \\
        --models-out LOG/models --vehicle x500 --speed 6 --from 270

Gazebo Harmonic, as PX4 1.16 sets it up, has no wind: PX4 doesn't load Gazebo's wind
system, and its models don't take part. This edits the run's own copies only:

  - the world (RUN.sdf): <wind><linear_velocity> in Gazebo's ENU, blowing from --from
    (degrees clockwise from north). Gazebo's lift-drag model (the cessna's and the
    VTOL's wings) reads it, so a plane flies through moving air: it crabs and its ground
    speed changes;
  - a multirotor has no aerodynamic model in Gazebo. For the x500, the run's server config
    also loads Gazebo's wind effects system and a copy of PX4's model with enable_wind on
    its base_link, found first on the resource path. That system pushes the body with
    mass x --scaling x (wind - velocity): a linear drag (0.1: about 1 m/s^2, 6 deg of lean,
    at 10 m/s through still air; a quadcopter's). An approximation of drag, not airflow.

The VTOL's hover phases feel no wind this way (no enable_wind on its body: its wings
already get the wind, and the drag would come on top of theirs in cruise).
"""
import argparse
import math
import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim_vehicles  # noqa: E402

WIND_SYSTEM = ('    <plugin entity_name="*" entity_type="world" filename="gz-sim-wind-effects-system" '
               'name="gz::sim::systems::WindEffects">\n'
               '      <force_approximation_scaling_factor>{scaling}</force_approximation_scaling_factor>\n'
               '    </plugin>\n')


def wind_enu(speed, from_deg):
    """(east, north) velocity of air blowing from from_deg."""
    to = math.radians(from_deg + 180.0)
    return speed * math.sin(to), speed * math.cos(to)


def set_world_wind(world_sdf, east, north):
    s = Path(world_sdf).read_text()
    s = re.sub(r"\s*<wind>.*?</wind>", "", s, flags=re.S)
    tag = f"    <wind>\n      <linear_velocity>{east:.6f} {north:.6f} 0</linear_velocity>\n    </wind>\n"
    i = s.rfind("</world>")
    if i < 0:
        sys.exit(f"sim_wind: no </world> in {world_sdf}")
    Path(world_sdf).write_text(s[:i] + tag + "  " + s[i:])


def add_wind_system(server_config, scaling):
    s = Path(server_config).read_text()
    if "WindEffects" in s:
        return
    i = s.rfind("</plugins>")
    if i < 0:
        sys.exit(f"sim_wind: no </plugins> in {server_config}")
    Path(server_config).write_text(s[:i] + WIND_SYSTEM.format(scaling=scaling) + "  " + s[i:])


def model_dirs():
    import os
    dirs = [ROOT / "sim/models"]
    px4 = os.environ.get("PX4_GZ_MODELS")
    if px4:
        dirs.append(Path(px4))
    for p in ("/opt/PX4-Autopilot", str(Path.home() / "PX4-Autopilot-1.16")):
        dirs.append(Path(p) / "Tools/simulation/gz/models")
    return [d for d in dirs if d.is_dir()]


def find_base_link_model(model):
    """The model directory whose model.sdf defines base_link, following <include>s from
    sim/models/<model> (x500_gdn -> x500 -> x500_base)."""
    seen = set()
    todo = [model]
    while todo:
        name = todo.pop(0)
        if name in seen or name == "gdn_payload":
            continue
        seen.add(name)
        for d in model_dirs():
            f = d / name / "model.sdf"
            if not f.exists():
                continue
            s = re.sub(r"<!--.*?-->", "", f.read_text(), flags=re.S)
            if re.search(r'<link name=["\']base_link["\']', s):
                return d / name
            todo += re.findall(r"<uri>model://([\w]+)</uri>", s)
            break
    sys.exit(f"sim_wind: no model with a base_link found from {model}")


def enable_wind_copy(model_dir, out_dir):
    dst = Path(out_dir) / model_dir.name
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(model_dir, dst)
    f = dst / "model.sdf"
    s = f.read_text()
    m = re.search(r'(<link name=["\']base_link["\']>)', s)
    s = s[:m.end()] + "\n      <enable_wind>true</enable_wind>" + s[m.end():]
    f.write_text(s)
    return dst


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--world", required=True)
    ap.add_argument("--server-config", required=True)
    ap.add_argument("--models-out", required=True)
    ap.add_argument("--vehicle", required=True)
    ap.add_argument("--speed", type=float, required=True)
    ap.add_argument("--from", dest="from_deg", type=float, required=True)
    ap.add_argument("--scaling", type=float, default=0.1)
    a = ap.parse_args()
    v = sim_vehicles.get(a.vehicle)
    e, n = wind_enu(a.speed, a.from_deg)
    set_world_wind(a.world, e, n)
    what = f"wind {a.speed:g} m/s from {a.from_deg:g} deg (ENU {e:+.2f} {n:+.2f} m/s) in {Path(a.world).name}"
    if v.kind == "multirotor":
        add_wind_system(a.server_config, a.scaling)
        Path(a.models_out).mkdir(parents=True, exist_ok=True)
        dst = enable_wind_copy(find_base_link_model(v.model), a.models_out)
        what += f"; wind effects (drag {a.scaling:g}/s) on {dst.name}'s base_link"
    else:
        what += "; through the wings' lift-drag model"
    print(f"sim_wind: {what}")


if __name__ == "__main__":
    main()
