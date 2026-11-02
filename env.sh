# GDN project environment. Source this in every project terminal:
#     source env.sh            (from the repo root, or with any path to it)
#
# Works wherever the repo is cloned; nothing assumes ~/gdn_workspace.

# Repo root = directory containing this file (bash and zsh).
if [ -n "${BASH_SOURCE[0]:-}" ]; then _gdn_self="${BASH_SOURCE[0]}"; else _gdn_self="${(%):-%x}"; fi
export GDN_ROOT="$(cd "$(dirname "$_gdn_self")" && pwd)"
unset _gdn_self

if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
else
  echo "[gdn] ROS 2 Jazzy not found at /opt/ros/jazzy - run tools/setup_ubuntu.sh first" >&2
fi

# Middleware. Fast DDS is the Jazzy default and is what the PX4 uXRCE-DDS
# agent talks to. Override with e.g. GDN_RMW=rmw_zenoh_cpp before sourcing.
export RMW_IMPLEMENTATION="${GDN_RMW:-rmw_fastrtps_cpp}"

# Keep this project's ROS graph separate from other work on the same LAN.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

if [ -f "$GDN_ROOT/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$GDN_ROOT/install/setup.bash"
else
  echo "[gdn] workspace not built yet - run: colcon build --symlink-install" >&2
fi
