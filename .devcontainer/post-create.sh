#!/usr/bin/env bash
#
# Runs once, when the container is first created.
#
# Everything here is idempotent and guarded, so rebuilding the container does
# not redo work that is already sitting in the bind mount. It does NOT clone
# PX4-Autopilot - that is 2 GB and a long build, so it lives in
# setup-px4.sh and is run deliberately.

set -euo pipefail

WORKSPACE="${HEGE_WS:-/workspaces/GPS-based-Navigation-for-Hege-Robot}"
cd "${WORKSPACE}"

# ROS setup scripts reference variables that may not exist yet, and this shell
# runs with nounset.
set +u
source /opt/ros/humble/setup.bash
set -u

echo "=============================================================="
echo " 1/4  px4_msgs"
echo "=============================================================="
# Not vendored: it is generated from the firmware's message definitions and has
# to match the PX4 version the board runs, so it is pinned in EXTERNAL_DEPS.txt
# and cloned rather than committed. A mismatch shows up as topics that exist
# and never deliver a message.
if [ -d "src/px4_msgs/.git" ]; then
  echo "already present, skipping"
else
  git clone --branch release/1.16 https://github.com/PX4/px4_msgs.git src/px4_msgs
  git -C src/px4_msgs checkout 392e831
fi

echo
echo "=============================================================="
echo " 2/4  gz_ros2_control, from source"
echo "=============================================================="
# ros-humble-gz-ros2-control from packages.ros.org depends on libsdformat12,
# which is Gazebo Fortress. Harmonic is libsdformat14, so that plugin will not
# load into gz-sim8 and the binary package is no use here. GZ_VERSION selects
# the Harmonic libraries at configure time.
#
# rosdep is deliberately not run in this workspace: it would resolve the Gazebo
# dependencies back to Fortress, which is the whole problem. The Harmonic
# headers come from gz-harmonic in the image.
if [ -f "${WORKSPACE}/gz_ros2_control_ws/install/setup.bash" ]; then
  echo "already built, skipping"
else
  mkdir -p "${WORKSPACE}/gz_ros2_control_ws/src"
  if [ ! -d "${WORKSPACE}/gz_ros2_control_ws/src/gz_ros2_control/.git" ]; then
    git clone --branch humble https://github.com/ros-controls/gz_ros2_control.git \
      "${WORKSPACE}/gz_ros2_control_ws/src/gz_ros2_control"
    git -C "${WORKSPACE}/gz_ros2_control_ws/src/gz_ros2_control" checkout c88a5fd
  fi
  cd "${WORKSPACE}/gz_ros2_control_ws"
  GZ_VERSION=harmonic colcon build --symlink-install
  cd "${WORKSPACE}"
fi

echo
echo "=============================================================="
echo " 3/4  building the workspace"
echo "=============================================================="
set +u
source "${WORKSPACE}/gz_ros2_control_ws/install/setup.bash"
set -u
# px4_msgs generates a lot of message code; the first build is slow.
colcon build --symlink-install

echo
echo "=============================================================="
echo " 4/4  shell setup"
echo "=============================================================="
add_line() {
  grep -qxF "$1" "${HOME}/.bashrc" || echo "$1" >> "${HOME}/.bashrc"
}
add_line 'set +u'
add_line 'source /opt/ros/humble/setup.bash'
add_line '[ -f "${HEGE_WS}/gz_ros2_control_ws/install/setup.bash" ] && source "${HEGE_WS}/gz_ros2_control_ws/install/setup.bash"'
add_line '[ -f "${HEGE_WS}/install/setup.bash" ] && source "${HEGE_WS}/install/setup.bash"'
add_line 'set -u'
echo "every new shell now sources ROS, gz_ros2_control and this workspace"

cat <<EOF

==============================================================
 Ready.

 What works right now:

   ros2 launch hege_description spawn_hege_gz.launch.py
       the navigation simulation on Gazebo Harmonic

   python3 -m pytest src/hege_px4_bridge/test src/hege_px4_sensors/test
       the bridge and sensor unit tests, 141 of them

 What needs PX4-Autopilot, which is not cloned yet:

   bash .devcontainer/setup-px4.sh
       clones PX4 v1.16.1, generates the Hege Gazebo model and
       registers the airframe. Around 2 GB and 20-40 minutes.

 Then:

   cd PX4-Autopilot && make px4_sitl gz_hege_rover
   ros2 launch hege_bringup hege_sitl.launch.py rviz:=true

 docs/px4_sitl.md has the detail.
==============================================================
EOF
