#!/usr/bin/env bash
# Look at an R2.2 site (sim/sites.yaml) in Gazebo: the terrain with its photo and the
# x500 on its pad. No PX4, no GDN, nothing flies.
#
#   tools/view_site.sh ch_zurich         # one site
#   tools/view_site.sh --all             # each site in turn: close the window for the next
#   tools/view_site.sh --list            # the sites
#
# In the dev image (the windows show on your desktop):
#   tools/docker_dev.sh shell --gpu --gui
#   tools/view_site.sh --all
#
# It builds a site first if its world is missing or out of date (tools/make_site.py,
# downloads cached). It runs in its own Gazebo partition, so it neither sees nor
# disturbs a simulation running on the same machine.
#
# In the window: left-drag pans, right-drag (or the wheel) zooms, middle-drag (or
# Shift + left-drag) orbits. It opens looking at the whole 4 km square from the
# south-west; the drone is on the pad in the middle. In the entity tree on the right,
# right-click x500_gdn_0, "Move to", to fly the camera down to it.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.16}"
die() { echo "view_site: $*" >&2; exit 1; }
SITES=$(python3 -c "import sys; sys.path.insert(0, 'tools'); import swiss_terrain as st; print(' '.join(st.load_sites()))")
case "${1:-}" in
  ""|-h|--help) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
  --list) python3 - <<'EOF'
import sys; sys.path.insert(0, "tools")
import swiss_terrain as st
for n, s in st.load_sites().items():
    print(f"{n:14s} {s['class']:22s} {s['place']}")
EOF
    exit 0 ;;
  --all) LIST=$SITES ;;
  *) LIST="$1"; [[ " $SITES " == *" $1 "* ]] || die "no site $1 (sites: $SITES)" ;;
esac

command -v gz >/dev/null || die "Gazebo (gz) not found: run this in the dev image (tools/docker_dev.sh shell --gpu --gui)"
[ -f "$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh" ] || die "PX4 SITL not built in $PX4_DIR (the x500 model comes from PX4)"

# Gazebo paths and plugins as PX4 sets them (the x500 and its systems), plus GDN's models.
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
set +u
# shellcheck disable=SC1091
source "$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh"
set -u
export GZ_IP="${GZ_IP:-127.0.0.1}"
export GZ_PARTITION="gdn_view_$$"          # apart from any running simulation

for SITE in $LIST; do
  python3 tools/make_site.py "$SITE" --if-stale || die "building $SITE failed"
  WDIR="$ROOT/data/sim/$SITE"
  python3 - "$SITE" <<'EOF'
import sys; sys.path.insert(0, "tools")
import swiss_terrain as st
s = st.load_sites()[sys.argv[1]]
print(f"view_site: {sys.argv[1]}: {s['class']}, {s['place']} (pad at {s['centre'][0]}, {s['centre'][1]})")
EOF
  echo "view_site: loading (the 4 km mesh takes 10 to 30 s); close the window to go on"
  # The GUI's own start camera is inside the terrain and its far clip too short
  # (tools/gz_gui_config.py).
  GUI_CFG="$(mktemp --suffix=.config)"
  python3 tools/gz_gui_config.py "$WDIR" "$GUI_CFG" || die "no GUI config"
  # VIEW_SITE_CHECK=1: load the world without a window and stop (what the check uses).
  GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$ROOT/sim/models:$WDIR" \
    gz sim -v "${VIEW_SITE_VERBOSE:-1}" --gui-config "$GUI_CFG" \
    ${VIEW_SITE_CHECK:+-s -r --headless-rendering --iterations 250} "$WDIR/$SITE.sdf"
  rm -f "$GUI_CFG"
done
