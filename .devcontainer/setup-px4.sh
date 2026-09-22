#!/usr/bin/env bash
#
# Clone PX4-Autopilot and set the Hege rover up inside it.
#
#     bash .devcontainer/setup-px4.sh
#
# Separate from post-create.sh because this is around 2 GB of checkout and a
# long first build, and not everyone opening the container needs it. Safe to
# re-run: every step is guarded.

set -euo pipefail

WORKSPACE="${HEGE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
PX4_DIR="${WORKSPACE}/PX4-Autopilot"
PX4_TAG="v1.16.1"

cd "${WORKSPACE}"

echo "=============================================================="
echo " 1/3  PX4-Autopilot ${PX4_TAG}"
echo "=============================================================="
# Inside the workspace rather than the home directory, so it survives a
# container rebuild - the bind mount persists, the container filesystem does
# not. PX4-Autopilot/ is in .gitignore.
#
# Tools/simulation/gz is a submodule and carries the Gazebo models PX4 spawns,
# so --recursive is not optional here.
if [ -d "${PX4_DIR}/.git" ]; then
  echo "already cloned, skipping"
else
  git clone --branch "${PX4_TAG}" --recursive \
    https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}"
fi

echo
echo "=============================================================="
echo " 2/3  the Hege model and airframe"
echo "=============================================================="
set +u
source /opt/ros/humble/setup.bash
set -u

# Generates models/hege_rover/model.sdf from hege.urdf.xacro with drive:=px4,
# then checks that the link and sensor names PX4 hardcodes survived the URDF to
# SDF conversion.
bash "${WORKSPACE}/src/hege_px4_sim/scripts/generate_hege_model.sh"

# Symlinks that model into the PX4 tree, installs 4100_gz_hege_rover and
# registers it in the airframe CMakeLists. --uninstall reverses all three.
bash "${WORKSPACE}/src/hege_px4_sim/scripts/install_to_px4.sh" "${PX4_DIR}"

echo
echo "=============================================================="
echo " 3/3  next"
echo "=============================================================="
cat <<EOF

Build and run PX4 SITL, in its own terminal:

    cd ${PX4_DIR}
    make px4_sitl gz_hege_rover

The first build is long - it compiles the firmware and regenerates the ROMFS
for the new airframe. Then, in a second terminal:

    ros2 launch hege_bringup hege_sitl.launch.py rviz:=true

Nothing moves until PX4 is armed and in offboard mode:

    ros2 service call /hege/bridge/set_offboard std_srvs/srv/Trigger
    ros2 service call /hege/bridge/arm std_srvs/srv/Trigger
    ros2 topic echo /hege/bridge/status --once

None of this has ever been run. docs/px4_sitl.md lists what to suspect first.
EOF
