#!/usr/bin/env python3
"""Write a Gazebo GUI config for one of our worlds: Gazebo's default config with a start
camera that shows the world, and a far clip that reaches across it.

    python3 tools/gz_gui_config.py data/sim/ch_zurich /tmp/gui.config
    gz sim -g --gui-config /tmp/gui.config

Why: Gazebo Harmonic's GUI ignores the world's <gui><camera> and starts at its own
default, 6 m above the world origin. On our worlds the origin is the lowest terrain
point, so the camera starts inside the hills, sees only sky, and the mesh (drawn from
above) is invisible. The GUI camera's default far clip is also too short for a 4 km
world seen from a few km away. Both are set in the 3D view plugin (MinimalScene).

Start camera: an R2.2 site (world_info has site_def) gets the whole square from the
south-west, 1.4 km above the pad; any other world gets a view close behind the pad.
"""
import math
import re
import sys
from pathlib import Path

DEFAULTS = ["/usr/share/gz/gz-sim8/gui/gui.config", str(Path.home() / ".gz/sim/8/gui.config")]


def camera_pose(info):
    pad_z = float(info["pad_z"])
    if "site_def" in info:
        side = max(float(info["size_e"]), float(info["size_n"]))
        d = 0.55 * side
        return f"{-d:.0f} {-d:.0f} {pad_z + 0.35 * side:.0f} 0 0.42 {math.pi / 4:.3f}"
    return f"-60 -60 {pad_z + 40:.1f} 0 0.4 {math.pi / 4:.3f}"


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    wdir, out = Path(sys.argv[1]), Path(sys.argv[2])
    info = dict(l.split("=", 1) for l in (wdir / "world_info.txt").read_text().split("\n") if "=" in l)
    src = next((Path(p) for p in DEFAULTS if Path(p).exists()), None)
    if src is None:
        sys.exit("gz_gui_config: Gazebo's default gui.config not found (is Gazebo Harmonic installed?)")
    text = src.read_text()
    scene = re.search(r'<plugin filename="MinimalScene".*?</plugin>', text, re.S)
    if scene is None:
        sys.exit(f"gz_gui_config: no MinimalScene plugin in {src}")
    block = re.sub(r"\s*<camera_pose>.*?</camera_pose>|\s*<camera_clip>.*?</camera_clip>", "", scene.group(0), flags=re.S)
    block = block.replace("</plugin>", f"  <camera_pose>{camera_pose(info)}</camera_pose>\n"
                                       "  <camera_clip><near>0.5</near><far>30000</far></camera_clip>\n</plugin>")
    out.write_text(text[:scene.start()] + block + text[scene.end():])


if __name__ == "__main__":
    main()
