# GDN dev image (R2.1): the pinned simulation toolchain.
#
#   Ubuntu 24.04 + ROS 2 Jazzy desktop  (base image, pinned by digest)
#   Gazebo Harmonic                      (PX4's setup script)
#   PX4 v1.16.2, SITL built              (/opt/PX4-Autopilot, pinned commit)
#   Micro XRCE-DDS Agent v2.4.3          (pinned commit)
#   ros_gz_bridge, Gazebo Python bindings, every GDN build and tool dependency
#
# The GDN workspace is not in the image: tools/docker_dev.sh mounts it at the same path
# as on the host, with the container's own build/ and install/. The user inside is the
# host user (same name, UID and GID), so files written in the workspace stay yours.
#
# Build and use it through tools/docker_dev.sh; pins are in docker/versions.env.
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC PIP_DEFAULT_TIMEOUT=120 PIP_RETRIES=10
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Survive a slow or unstable connection: apt retries each download, git uses HTTP/1.1
# (long HTTP/2 transfers from GitHub were cut: "RPC failed ... early EOF") and gives up
# on a stalled transfer so docker/retry.sh can start it again.
COPY docker/retry.sh /usr/local/bin/retry
RUN echo 'Acquire::Retries "10";' > /etc/apt/apt.conf.d/80-retries \
    && git config --system http.version HTTP/1.1 \
    && git config --system http.lowSpeedLimit 1000 \
    && git config --system http.lowSpeedTime 120

RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo git ca-certificates wget curl lsb-release gnupg \
    && rm -rf /var/lib/apt/lists/*

# PX4 at the pinned commit, and its dependencies (including Gazebo Harmonic) from
# PX4's own setup script, as tools/setup_sim.sh does on a host.
ARG PX4_TAG
ARG PX4_COMMIT
RUN retry -c /opt/PX4-Autopilot git clone --branch "${PX4_TAG}" --depth 1 \
      https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
    && test "$(git -C /opt/PX4-Autopilot rev-parse HEAD)" = "${PX4_COMMIT}" \
    && retry git -C /opt/PX4-Autopilot submodule update --init --recursive --depth 1 --jobs 4
# PX4's script pip-installs its Python requirements as root. pip then tries to upgrade
# packages Ubuntu installed (numpy 1.26.4, which ROS 2 Jazzy is built against), and
# can't remove them ("Cannot uninstall numpy 1.26.4, RECORD file not found"). Pin
# every Python package already installed to its version, so pip picks versions that
# work with them instead of replacing them.
RUN python3 -c "import importlib.metadata as m; \
print('\n'.join(sorted({f'{d.metadata[\"Name\"]}=={d.version}' for d in m.distributions() if d.metadata['Name']})))" \
      > /opt/pip-constraints.txt \
    && grep -c . /opt/pip-constraints.txt
ENV PIP_CONSTRAINT=/opt/pip-constraints.txt
RUN USER=root retry -n 3 bash /opt/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx \
    && rm -rf /var/lib/apt/lists/*

# ROS-Gazebo bridge, Gazebo Python bindings, and what GDN builds and runs with (the
# same list as tools/setup_ubuntu.sh and CI), then rosdep for GDN's packages.
COPY src/ /tmp/gdn_pkgs/
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz-bridge \
      python3-gz-sim8 python3-gz-math7 python3-gz-transport13 python3-gz-msgs10 \
      ros-dev-tools ros-jazzy-rosbag2-storage-mcap ros-jazzy-rclcpp-components \
      build-essential cmake git libeigen3-dev pkg-config \
      clang clang-tidy clang-format cppcheck \
      python3-numpy python3-scipy python3-matplotlib python3-h5py \
      python3-yaml python3-pil python3-rasterio \
    && retry rosdep update --rosdistro jazzy \
    && rosdep install --from-paths /tmp/gdn_pkgs --ignore-src -y --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/* /tmp/gdn_pkgs

# Micro XRCE-DDS Agent at the pinned commit (the PX4 <-> ROS 2 link).
ARG XRCE_TAG
ARG XRCE_COMMIT
RUN retry -c /tmp/agent git clone --branch "${XRCE_TAG}" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent \
    && test "$(git -C /tmp/agent rev-parse HEAD)" = "${XRCE_COMMIT}" \
    && cmake -S /tmp/agent -B /tmp/agent/build -DCMAKE_BUILD_TYPE=Release \
    && retry cmake --build /tmp/agent/build -j"$(nproc)" \
    && cmake --install /tmp/agent/build \
    && ldconfig /usr/local/lib \
    && mkdir -p /opt/gdn && echo "${XRCE_COMMIT}" > /opt/gdn/xrce_commit \
    && rm -rf /tmp/agent

# The host user. Ubuntu 24.04 images ship a user "ubuntu" with UID 1000, which is
# usually the host user's UID too, so it is removed first.
ARG USERNAME=gdn
ARG USER_UID=1000
ARG USER_GID=1000
RUN if getent passwd "${USER_UID}" >/dev/null; then userdel -r "$(getent passwd "${USER_UID}" | cut -d: -f1)"; fi \
    && if ! getent group "${USER_GID}" >/dev/null; then groupadd -g "${USER_GID}" "${USERNAME}"; fi \
    && useradd -m -u "${USER_UID}" -g "${USER_GID}" -s /bin/bash "${USERNAME}" \
    && usermod -aG dialout,video "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chown -R "${USER_UID}:${USER_GID}" /opt/PX4-Autopilot

# PX4 SITL, built as that user (PX4 writes into its build tree at run time).
#
# PX4's version header (px_update_git_header.py) reads the NuttX submodule's tags, and
# takes the highest "nuttx-X.Y.Z". The shallow submodule clone has no tags, and
# fetching them would download a whole tree per tag. So list the tag names from the
# remote (a few KB) and create them locally on the checked-out commit: the script uses
# only the names and HEAD's hash, so the header comes out as from a full clone.
USER ${USERNAME}
RUN cd /opt/PX4-Autopilot/platforms/nuttx/NuttX/nuttx \
    && retry bash -c "git ls-remote --tags --refs origin 'nuttx-*' > /tmp/nuttx_tags" \
    && cut -f2 /tmp/nuttx_tags | sed 's#refs/tags/##' | xargs -r -n1 git tag -f >/dev/null \
    && git -c versionsort.suffix=- tag --sort=v:refname | grep -E '^nuttx-[0-9]+\.[0-9]+\.[0-9]+' | tail -1 \
    && cd /opt/PX4-Autopilot && make px4_sitl
ENV PX4_DIR=/opt/PX4-Autopilot

USER root
COPY docker/entrypoint.sh /opt/gdn/entrypoint.sh
RUN chmod 755 /opt/gdn/entrypoint.sh && dpkg-query -W > /opt/gdn/packages.txt

# Declared last: a build arg that changes invalidates the cache from where it is
# declared, and the hash changes with every edit of the docker/ files.
ARG BASE_IMAGE
ARG DOCKER_HASH
LABEL gdn.docker_hash="${DOCKER_HASH}" gdn.base_image="${BASE_IMAGE}" gdn.px4="${PX4_TAG}" gdn.xrce="${XRCE_TAG}"
USER ${USERNAME}
ENTRYPOINT ["/opt/gdn/entrypoint.sh"]
CMD ["bash"]
