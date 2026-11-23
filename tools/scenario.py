#!/usr/bin/env python3
"""SITL scenarios (roadmap R2.6): the file format and its reader.

A scenario is a YAML file (config/scenarios/<name>.yaml) naming one simulated flight.
tools/run_scenario.py flies it; tools/check_scenario.py checks a run did what it asked.

    schema: 1
    name: x500_emmental_gnss_cut      # the file's name
    description: free text
    vehicle: x500                     # tools/sim_vehicles.py: x500 | standard_vtol | rc_cessna
    world: ch_emmental                # an R2.2 site (sim/sites.yaml), or zermatt
    route:                            # tools/sim_fly.py's lawnmower, centred on the pad
      legs: 2
      leg_len_m: 800
      spacing_m: 250                  # optional, default 250
      clearance_m: 40                 # optional: height above the terrain envelope (default 60)
      speed_mps: null                 # optional: null = the vehicle's (x500 10, planes 15)
      launch: runway                  # optional, fixed-wing: runway | catapult
    gnss:
      loss: none                      # none | cut (PX4's EKF2_GPS_CTRL 0, as Stage B)
      at_s: 120                       # cut: seconds after reaching cruise height
    start_error:                      # optional: GDN starts this far from the truth
      distance_m: 0                   #   (once heading is aligned, as mc_campaign --start-error)
      bearing_deg: 0                  #   the direction from the truth to GDN's first estimate
    wind:                             # optional: a steady wind (Gazebo's wind effects)
      speed_mps: 0
      from_deg: 270                   #   where it blows from, clockwise from north
    limits:
      timeout_s: 1800                 # the whole run, start to teardown

Every key is checked: unknown keys, missing required keys and values out of range are
errors, with the file and key named. R2.6b adds spoofing (gnss.loss: spoof), sensor
faults and lighting.

    python3 tools/scenario.py config/scenarios/x500_emmental_gnss_cut.yaml   # read and print
    python3 tools/scenario.py --list                                          # every scenario
    python3 tools/scenario.py --check                                         # all read cleanly (CI)
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = 1


class ScenarioError(ValueError):
    pass


def _vehicles():
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import sim_vehicles
    return sim_vehicles.VEHICLES


def _worlds():
    import yaml
    sites = yaml.safe_load((ROOT / "sim/sites.yaml").read_text())
    names = set((sites.get("sites") or {}).keys()) if isinstance(sites, dict) else set()
    return names | {"zermatt"}


def _num(d, key, where, lo=None, hi=None, default=None, required=True, integer=False, allow_null=False):
    if key not in d:
        if required:
            raise ScenarioError(f"{where}: missing {key}")
        return default
    v = d[key]
    if v is None and allow_null:
        return None
    if isinstance(v, bool) or not isinstance(v, (int, float)) or (integer and not isinstance(v, int)):
        raise ScenarioError(f"{where}.{key}: {'an integer' if integer else 'a number'} expected (is {v!r})")
    if not math.isfinite(v) or (lo is not None and v < lo) or (hi is not None and v > hi):
        raise ScenarioError(f"{where}.{key}: {v} is outside {lo} to {hi}")
    return v


def _keys(d, where, allowed):
    if not isinstance(d, dict):
        raise ScenarioError(f"{where}: a mapping expected")
    extra = sorted(set(d) - set(allowed))
    if extra:
        raise ScenarioError(f"{where}: unknown key {extra[0]} (known: {', '.join(allowed)})")


def parse(data, label="scenario", check_names=True):
    """A checked scenario as a plain dict with every default filled in."""
    top = ("schema", "name", "description", "vehicle", "world", "route", "gnss", "start_error", "wind", "limits")
    _keys(data, label, top)
    for k in ("schema", "name", "vehicle", "world", "route", "gnss", "limits"):
        if k not in data:
            raise ScenarioError(f"{label}: missing {k}")
    if data["schema"] != SCHEMA:
        raise ScenarioError(f"{label}: schema {data['schema']}, this reads schema {SCHEMA}")
    out = {"schema": SCHEMA, "name": str(data["name"]), "description": str(data.get("description", ""))}
    if check_names:
        if data["vehicle"] not in _vehicles():
            raise ScenarioError(f"{label}.vehicle: {data['vehicle']} (known: {', '.join(_vehicles())})")
        if data["world"] not in _worlds():
            raise ScenarioError(f"{label}.world: {data['world']} (known: {', '.join(sorted(_worlds()))})")
    out["vehicle"], out["world"] = data["vehicle"], data["world"]
    r = data["route"]
    _keys(r, f"{label}.route", ("legs", "leg_len_m", "spacing_m", "clearance_m", "speed_mps", "launch"))
    w = f"{label}.route"
    out["route"] = {"legs": _num(r, "legs", w, 1, 40, integer=True), "leg_len_m": _num(r, "leg_len_m", w, 50, 3600),
                    "spacing_m": _num(r, "spacing_m", w, 20, 1000, default=250.0, required=False),
                    "clearance_m": _num(r, "clearance_m", w, 10, 500, default=60.0, required=False),
                    "speed_mps": _num(r, "speed_mps", w, 2, 40, required=False, allow_null=True),
                    "launch": r.get("launch", "runway")}
    if out["route"]["launch"] not in ("runway", "catapult"):
        raise ScenarioError(f"{w}.launch: runway or catapult (is {out['route']['launch']!r})")
    half = 0.5 * out["route"]["leg_len_m"]
    across = 0.5 * (out["route"]["legs"] - 1) * out["route"]["spacing_m"]
    if half > 1800 or across > 1800:
        raise ScenarioError(f"{w}: the pattern reaches {max(half, across):.0f} m from the pad; the worlds are "
                            "4 km across (keep it within 1800 m)")
    g = data["gnss"]
    _keys(g, f"{label}.gnss", ("loss", "at_s"))
    if g.get("loss") not in ("none", "cut"):
        raise ScenarioError(f"{label}.gnss.loss: none or cut (spoofing comes with R2.6b; is {g.get('loss')!r})")
    out["gnss"] = {"loss": g["loss"],
                   "at_s": _num(g, "at_s", f"{label}.gnss", 0, 7200) if g["loss"] != "none" else None}
    se = data.get("start_error") or {}
    _keys(se, f"{label}.start_error", ("distance_m", "bearing_deg"))
    out["start_error"] = {"distance_m": _num(se, "distance_m", f"{label}.start_error", 0, 2000, default=0.0,
                                             required=False),
                          "bearing_deg": _num(se, "bearing_deg", f"{label}.start_error", -360, 360, default=0.0,
                                              required=False)}
    wi = data.get("wind") or {}
    _keys(wi, f"{label}.wind", ("speed_mps", "from_deg"))
    out["wind"] = {"speed_mps": _num(wi, "speed_mps", f"{label}.wind", 0, 20, default=0.0, required=False),
                   "from_deg": _num(wi, "from_deg", f"{label}.wind", -360, 360, default=0.0, required=False)}
    lim = data["limits"]
    _keys(lim, f"{label}.limits", ("timeout_s",))
    out["limits"] = {"timeout_s": _num(lim, "timeout_s", f"{label}.limits", 120, 14400)}
    return out


def load(path):
    import yaml
    p = Path(path)
    if not p.is_absolute() and not p.exists():
        p = ROOT / p
    try:
        data = yaml.safe_load(p.read_text())
    except yaml.YAMLError as e:
        raise ScenarioError(f"{path}: not valid YAML: {e}") from e
    s = parse(data, str(path))
    if s["name"] != p.stem:
        raise ScenarioError(f"{path}: name {s['name']!r} isn't the file's ({p.stem})")
    return s


def sim_env(s):
    """The tools/sim_run.sh settings that fly this scenario."""
    r = s["route"]
    fly = [f"--legs {r['legs']}", f"--leg-len {r['leg_len_m']:g}", f"--spacing {r['spacing_m']:g}",
           f"--clearance {r['clearance_m']:g}"]
    if r["speed_mps"] is not None:
        fly.append(f"--speed {r['speed_mps']:g}")
    env = {"GDN_VEHICLE": s["vehicle"], "GDN_FLY_ARGS": " ".join(fly), "HEADLESS": "1"}
    if s["world"] == "zermatt":
        env["GDN_SIM_WORLD"] = "zermatt"
    else:
        env["GDN_SITE"] = s["world"]
    if r["launch"] == "catapult":
        env["GDN_FW_LAUNCH"] = "catapult"
    if s["gnss"]["loss"] == "cut":
        env["GPS_OFF_AT"] = f"{s['gnss']['at_s']:g}"
    se = s["start_error"]
    if se["distance_m"] > 0:
        b = math.radians(se["bearing_deg"])
        env["GDN_START_ERROR"] = f"{se['distance_m'] * math.cos(b):.3f} {se['distance_m'] * math.sin(b):.3f}"
    wi = s["wind"]
    if wi["speed_mps"] > 0:
        env["GDN_WIND"] = f"{wi['speed_mps']:g} {wi['from_deg']:g}"
    return env


def main():
    if len(sys.argv) == 2 and sys.argv[1] == "--check":     # every scenario file (CI)
        files = sorted((ROOT / "config/scenarios").glob("*.yaml"))
        bad = 0
        for f in files:
            try:
                load(f)
                print(f"PASS  {f.relative_to(ROOT)}")
            except ScenarioError as e:
                print(f"FAIL  {e}")
                bad += 1
        print(f"{len(files) - bad} of {len(files)} scenarios read cleanly")
        sys.exit(1 if bad or not files else 0)
    if len(sys.argv) == 2 and sys.argv[1] == "--list":
        for f in sorted((ROOT / "config/scenarios").glob("*.yaml")):
            try:
                s = load(f)
                print(f"{s['name']:36s} {s['vehicle']:14s} {s['world']:13s} gnss {s['gnss']['loss']:5s} "
                      f"start {s['start_error']['distance_m']:g} m, wind {s['wind']['speed_mps']:g} m/s")
            except ScenarioError as e:
                print(f"ERROR {e}")
        return
    if len(sys.argv) != 2 or sys.argv[1].startswith("-"):
        sys.exit(__doc__)
    try:
        s = load(sys.argv[1])
    except (ScenarioError, OSError) as e:
        sys.exit(f"ERROR: {e}")
    import json
    print(json.dumps(s, indent=1))
    print("sim_run.sh settings: " + " ".join(f'{k}="{v}"' for k, v in sim_env(s).items()))


if __name__ == "__main__":
    main()
