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

# Gazebo's command line ("gz sim", "gz topic") finds its commands through
# GZ_CONFIG_PATH. ROS 2 sets it to its own gz_*_vendor folders only; where ROS has
# some of those packages but not gz_sim_vendor (the dev image, R2.1), the system
# Gazebo Harmonic's "sim" command in /usr/share/gz is hidden and "gz sim" just prints
# help. Add the system folder after ROS's.
if [ -d /usr/share/gz ] && [[ ":${GZ_CONFIG_PATH:-}:" != *":/usr/share/gz:"* ]]; then
  export GZ_CONFIG_PATH="${GZ_CONFIG_PATH:+$GZ_CONFIG_PATH:}/usr/share/gz"
fi

# Middleware. Fast DDS is the Jazzy default and is what the PX4 uXRCE-DDS
# agent talks to. Override with e.g. GDN_RMW=rmw_zenoh_cpp before sourcing.
export RMW_IMPLEMENTATION="${GDN_RMW:-rmw_fastrtps_cpp}"

# Hybrid-graphics laptops (Intel + NVIDIA, "on-demand" PRIME): the desktop runs on
# the Intel GPU, and programs only use the NVIDIA GPU when asked. Gazebo's camera
# and lidar sensors need the NVIDIA GPU, so set GDN_GPU=nvidia before sourcing
# this file in simulation terminals:  GDN_GPU=nvidia source env.sh
if [ "${GDN_GPU:-}" = "nvidia" ]; then
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __VK_LAYER_NV_optimus=NVIDIA_only
  if [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]; then
    # headless rendering (gz sim -s --headless-rendering) goes through EGL
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  fi
fi

# Keep this project's ROS graph separate from other work on the same LAN.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

if [ -f "$GDN_ROOT/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$GDN_ROOT/install/setup.bash"
else
  echo "[gdn] workspace not built yet - run: colcon build --symlink-install" >&2
fi
