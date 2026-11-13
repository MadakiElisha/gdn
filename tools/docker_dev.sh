#!/usr/bin/env bash
# The GDN dev image (R2.1): the pinned simulation toolchain in Docker, so every machine
# runs the same PX4, Gazebo and ROS 2.
#
#   tools/docker_dev.sh build              # build the image (30 to 40 min the first time)
#   tools/docker_dev.sh shell              # a shell in it, in this workspace
#   tools/docker_dev.sh shell --gpu        # with the NVIDIA GPU (Gazebo's rendered sensors)
#   tools/docker_dev.sh shell --gpu --gui  # and windows on your desktop (Gazebo, RViz)
#   tools/docker_dev.sh run [--gpu] [--gui] -- CMD ARGS...   # one command
#   tools/docker_dev.sh image | hash       # the image name / the hash of the docker/ files
#
# Inside: the workspace is mounted at the same path as here, and the container builds
# into its own build/ and install/ (kept in .docker/ here), so a host build and a
# container build never mix. log/ is shared, so run logs land where the host sees them.
# The network is the host's, so ROS 2 topics and Gazebo pass between host and container.
#
# Pins: docker/versions.env. Check the image: tools/docker_check.sh [--gpu].
# --gpu needs the NVIDIA Container Toolkit (nvidia-ctk) on the host.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
# shellcheck disable=SC1091
. docker/versions.env

docker_hash() {   # what the image is built from: the docker/ files and the package manifests
  { find docker -type f | LC_ALL=C sort; find src -mindepth 2 -maxdepth 2 -name package.xml | LC_ALL=C sort; } \
    | while read -r f; do echo "$f"; cat "$f"; done | sha256sum | cut -c1-12
}
image() { echo "gdn-dev:$(docker_hash)"; }

usage() { sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }

cmd="${1:-}"; [ $# -gt 0 ] && shift
case "$cmd" in
  hash) docker_hash; exit 0 ;;
  image) image; exit 0 ;;
  build)
    h="$(docker_hash)"
    echo "docker_dev: building gdn-dev:$h (PX4 $PX4_TAG, base ${BASE_IMAGE%%@*})"
    docker build -f docker/Dockerfile \
      --build-arg BASE_IMAGE="$BASE_IMAGE" \
      --build-arg PX4_TAG="$PX4_TAG" --build-arg PX4_COMMIT="$PX4_COMMIT" \
      --build-arg XRCE_TAG="$XRCE_TAG" --build-arg XRCE_COMMIT="$XRCE_COMMIT" \
      --build-arg USERNAME="$(id -un)" --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)" \
      --build-arg DOCKER_HASH="$h" \
      -t "gdn-dev:$h" -t gdn-dev:latest "$@" .
    echo "docker_dev: built gdn-dev:$h. Check it: tools/docker_check.sh"
    exit 0 ;;
  shell|run) ;;
  *) usage ;;
esac

GPU=""; GUI=""
while [ $# -gt 0 ]; do
  case "$1" in
    --gpu) GPU=1; shift ;;
    --gui) GUI=1; shift ;;
    --) shift; break ;;
    *) [ "$cmd" = run ] && break; echo "unknown option $1"; exit 2 ;;
  esac
done
[ "$cmd" = shell ] && set -- bash
[ $# -gt 0 ] || { echo "run: give a command after --"; exit 2; }

IMG="$(image)"
docker image inspect "$IMG" >/dev/null 2>&1 || {
  echo "docker_dev: no image $IMG for the current docker/ files. Build it: tools/docker_dev.sh build" >&2; exit 1; }

# Mount points made here first, so Docker doesn't create them as root.
mkdir -p "$ROOT/.docker/build" "$ROOT/.docker/install" "$ROOT/build" "$ROOT/install" "$ROOT/log"
ARGS=(--rm --init --network host --ipc host --hostname gdn-dev
      -e GDN_ROOT="$ROOT" -e GDN_HOST_UID="$(id -u)" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
      -v "$ROOT:$ROOT" -v "$ROOT/.docker/build:$ROOT/build" -v "$ROOT/.docker/install:$ROOT/install"
      -w "$ROOT")
[ -t 0 ] && [ -t 1 ] && ARGS+=(-it)
if [ -n "$GPU" ]; then
  command -v nvidia-ctk >/dev/null || echo "docker_dev: warning, no nvidia-ctk: install the NVIDIA Container Toolkit for --gpu" >&2
  # "all" capabilities: graphics (EGL/GLX for Gazebo's rendered sensors) as well as compute.
  ARGS+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e GDN_GPU=nvidia)
fi
if [ -n "$GUI" ]; then
  # X11 (XWayland on a Wayland desktop). Allow only this user's local clients.
  command -v xhost >/dev/null && xhost "+SI:localuser:$(id -un)" >/dev/null
  ARGS+=(-e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix)
  [ -d /dev/dri ] && ARGS+=(--device /dev/dri)
fi
exec docker run "${ARGS[@]}" "$IMG" "$@"
