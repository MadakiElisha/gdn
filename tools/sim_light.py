#!/usr/bin/env python3
"""Lighting for one SITL run (roadmap R2.6b; tools/sim_run.sh calls it for GDN_LIGHTING).

    python3 tools/sim_light.py --world RUN.sdf --elevation 8 --azimuth 250 --intensity 0.3

Edits the run's own copy of the world:
  - the sun (the directional light named "sun") shines from --elevation degrees above
    the horizon, at --azimuth degrees clockwise from north (where the sun is);
  - its diffuse and specular colours, the scene's ambient light and the sky's background
    colour are scaled by --intensity (1 = the world's own sun; dusk about 0.2 to 0.3).

The worlds' own sun (tools/make_sim_world.py) is at about 61 deg elevation, azimuth
127 deg (south-east), diffuse 0.9 0.9 0.85, ambient 0.4.

It changes what the cameras see (the OAK-D Lite: tools/sim_adapter.py reports its mean
image brightness). GDN doesn't use the cameras until Phase 3 (VIO), so here it's a
scenario dimension for later, checked by its effect on the images.
"""
import argparse
import math
import re
import sys
from pathlib import Path


def sun_direction(elevation_deg, azimuth_deg):
    """Gazebo ENU direction the light travels (from the sun to the ground)."""
    el, az = math.radians(elevation_deg), math.radians(azimuth_deg)
    return (-math.cos(el) * math.sin(az), -math.cos(el) * math.cos(az), -math.sin(el))


def sun_from_direction(d):
    """(elevation, azimuth) degrees of a Gazebo light direction."""
    n = math.sqrt(sum(x * x for x in d))
    e, nn, u = (-x / n for x in d)
    return math.degrees(math.asin(u)), math.degrees(math.atan2(e, nn)) % 360.0


def _scale(colour, k):
    v = [float(x) for x in colour.split()]
    return " ".join(f"{min(1.0, c * k):.4f}" for c in v[:3]) + (f" {v[3]:g}" if len(v) > 3 else "")


def set_lighting(world_sdf, elevation, azimuth, intensity):
    s = Path(world_sdf).read_text()
    m = re.search(r'<light name="sun" type="directional">.*?</light>', s, flags=re.S)
    if not m:
        sys.exit(f"sim_light: no directional light named sun in {world_sdf}")
    light = m.group(0)
    d = sun_direction(elevation, azimuth)
    light = re.sub(r"<direction>[^<]*</direction>", f"<direction>{d[0]:.6f} {d[1]:.6f} {d[2]:.6f}</direction>", light)
    for tag in ("diffuse", "specular"):
        light = re.sub(rf"<{tag}>([^<]*)</{tag}>", lambda mm, t=tag: f"<{t}>{_scale(mm.group(1), intensity)}</{t}>",
                       light)
    s = s[:m.start()] + light + s[m.end():]
    for tag in ("ambient", "background"):
        s = re.sub(rf"(<scene>.*?)<{tag}>([^<]*)</{tag}>",
                   lambda mm, t=tag: f"{mm.group(1)}<{t}>{_scale(mm.group(2), intensity)}</{t}>", s, count=1, flags=re.S)
    Path(world_sdf).write_text(s)


def read_lighting(world_sdf):
    """(elevation, azimuth, diffuse rgb, ambient rgb) of a world's sun, or None."""
    s = Path(world_sdf).read_text()
    m = re.search(r'<light name="sun" type="directional">(.*?)</light>', s, flags=re.S)
    if not m:
        return None
    d = [float(x) for x in re.search(r"<direction>([^<]*)</direction>", m.group(1)).group(1).split()]
    dif = [float(x) for x in re.search(r"<diffuse>([^<]*)</diffuse>", m.group(1)).group(1).split()][:3]
    amb = re.search(r"<scene>.*?<ambient>([^<]*)</ambient>", s, flags=re.S)
    el, az = sun_from_direction(d)
    return el, az, dif, [float(x) for x in amb.group(1).split()][:3] if amb else None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--world", required=True)
    ap.add_argument("--elevation", type=float, required=True)
    ap.add_argument("--azimuth", type=float, required=True)
    ap.add_argument("--intensity", type=float, required=True)
    a = ap.parse_args()
    set_lighting(a.world, a.elevation, a.azimuth, a.intensity)
    el, az, dif, amb = read_lighting(a.world)
    print(f"sim_light: sun at {el:.1f} deg elevation, azimuth {az:.1f} deg, diffuse {dif[0]:.2f}, "
          f"ambient {amb[0] if amb else float('nan'):.2f} (x{a.intensity:g}) in {Path(a.world).name}")


if __name__ == "__main__":
    main()
