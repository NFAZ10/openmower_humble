ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ARG ROS_DISTRO
ARG WORKSPACE=/opt/openmowernext
ARG OPENMOWERNEXT_REPO=https://github.com/NFAZ10/openmowernext.git
ARG OPENMOWERNEXT_REF=3eb9edb2f8d29f5e28fdf242de78332c192d787a

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV WORKSPACE=${WORKSPACE}
ENV OPENMOWERNEXT_DIR=${WORKSPACE}/src/open_mower_next

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    patch \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-paho-mqtt \
    python3-pip \
    python3-pymongo \
    python3-rosdep \
    python3-vcstool \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir websockets==10.4

RUN if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && rosdep update

RUN mkdir -p "${WORKSPACE}/src" \
    && git clone "${OPENMOWERNEXT_REPO}" "${OPENMOWERNEXT_DIR}" \
    && git -C "${OPENMOWERNEXT_DIR}" checkout "${OPENMOWERNEXT_REF}"

COPY scripts/apply-humble-adaptation.py /usr/local/bin/apply-humble-adaptation.py
COPY scripts/openmower-ui-bridge.py /usr/local/bin/openmower-ui-bridge.py

RUN python3 /usr/local/bin/apply-humble-adaptation.py "${OPENMOWERNEXT_DIR}"

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd "${WORKSPACE}" \
    && mkdir -p src/lib \
    && vcs import src/lib --force --shallow < "${OPENMOWERNEXT_DIR}/custom_deps.yaml"

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd "${WORKSPACE}" \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" \
         --skip-keys="vesc_hw_interface gz_ros2_control ros_gz ros_gz_sim foxglove_bridge" \
         -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd "${WORKSPACE}" \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY scripts/docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh

RUN chmod +x /usr/local/bin/docker-entrypoint.sh \
    /usr/local/bin/openmower-ui-bridge.py \
    && mkdir -p /config /data

WORKDIR ${WORKSPACE}

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["ros2", "launch", "open_mower_next", "openmower.launch.py"]
