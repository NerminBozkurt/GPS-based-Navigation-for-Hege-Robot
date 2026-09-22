# ROS 2 Humble + Gazebo Harmonic + PX4 v1.16 build environment.
#
# This image exists because Gazebo Classic and Gazebo Harmonic cannot be
# installed on the same machine - their Debian packages both ship /usr/bin/gz
# and conflict outright. PX4 SITL requires Harmonic, so the PX4 work happens in
# here while the host keeps Classic and the navigation simulation.
#
# Deliberately NOT installed: anything Gazebo Classic. Installing
# ros-humble-gazebo-ros-pkgs in this image would remove Harmonic and defeat the
# whole point.
#
# Based on the devcontainer Oguzhan Enes Isik wrote for
# oguzissik/hege_gps_navigation, with three additions this project needs:
# the Micro XRCE-DDS Agent, the Harmonic build of ros_gz, and a source build of
# gz_ros2_control (see post-create.sh for why the binary one is no use).

FROM ros:humble-ros-base-jammy

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

# Two groups here: the PX4 firmware build toolchain (astyle, ninja, cppcheck,
# the rest of what Tools/setup/ubuntu.sh asks for) and the ROS packages this
# repository's own launch files need.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        astyle \
        build-essential \
        ca-certificates \
        ccache \
        cmake \
        cppcheck \
        file \
        g++ \
        gcc \
        gdb \
        gnupg \
        lcov \
        libfuse2 \
        libxml2-dev \
        libxml2-utils \
        lsb-release \
        make \
        ninja-build \
        pkg-config \
        python3-dev \
        python3-setuptools \
        python3-wheel \
        rsync \
        shellcheck \
        unzip \
        zip \
        bash-completion \
        can-utils \
        curl \
        git \
        iproute2 \
        nano \
        net-tools \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        sudo \
        usbutils \
        ros-humble-desktop \
        ros-humble-diagnostic-updater \
        ros-humble-geodesy \
        ros-humble-geographic-msgs \
        ros-humble-imu-tools \
        ros-humble-joint-state-publisher \
        ros-humble-nav2-bringup \
        ros-humble-navigation2 \
        ros-humble-robot-localization \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-tf-transformations \
        ros-humble-tf2-tools \
        ros-humble-twist-mux \
        ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic, from the OSRF repository. The same repository also carries
# ros_gz built against Harmonic, under a suffixed name - the ROS 2 Humble
# binaries (ros-humble-ros-gz-*) are built against Gazebo FORTRESS, which is
# Humble's official pairing, and would pull in a second conflicting Gazebo.
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-humble-ros-gzharmonic-sim \
        ros-humble-ros-gzharmonic-bridge \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# The Micro XRCE-DDS Agent: the process that carries PX4's uXRCE-DDS traffic
# onto the ROS 2 network. Without it /fmu/out/* topics simply never appear, and
# both hege_px4_bridge and hege_px4_sensors sit there with nothing to read.
# Built from source because that is what the PX4 documentation specifies.
RUN git clone --depth 1 --branch v2.4.2 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/uxrce \
    && cmake -S /tmp/uxrce -B /tmp/uxrce/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/uxrce/build -j"$(nproc)" \
    && cmake --install /tmp/uxrce/build \
    && ldconfig /usr/local/lib/ \
    && rm -rf /tmp/uxrce

# Python modules the PX4 v1.16.1 build system needs. Fetched from the pinned
# tag so the list matches the firmware that actually gets built.
RUN git clone --depth 1 --branch v1.16.1 \
        https://github.com/PX4/PX4-Autopilot.git /tmp/px4-setup \
    && python3 -m pip install --no-cache-dir \
        -r /tmp/px4-setup/Tools/setup/requirements.txt \
    && rm -rf /tmp/px4-setup

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}" \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

ENV DEBIAN_FRONTEND=dialog
ENV ROS_DISTRO=humble

USER ${USERNAME}
CMD ["bash"]
