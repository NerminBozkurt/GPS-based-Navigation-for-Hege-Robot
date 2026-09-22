#!/usr/bin/env bash
#
# Build the Gazebo Harmonic model PX4 SITL spawns for HEGE.
#
#     hege.urdf.xacro  --(xacro drive:=px4)-->  hege_px4.urdf  --(gz sdf -p)-->  model.sdf
#
# The xacro is the single source of truth for the vehicle's geometry, so this
# runs again whenever a measurement changes. model.sdf is a build artefact and
# is deliberately not committed - regenerating it is this script.
#
# Needs: xacro (ros-humble-xacro) and gz (gz-harmonic), both present in the
# devcontainer. Run it from anywhere.

set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg="$(dirname "${here}")"
repo="$(cd "${pkg}/../.." && pwd)"

xacro_file="${repo}/src/hege_description/urdf/hege.urdf.xacro"
model_dir="${pkg}/models/hege_rover"
urdf_out="$(mktemp -t hege_px4_XXXXXX.urdf)"
trap 'rm -f "${urdf_out}"' EXIT

# The model directory name and the <model name=...> both have to be
# hege_rover: PX4 derives the directory from PX4_SIM_MODEL=gz_hege_rover and
# spawns the file at ${PX4_GZ_MODELS}/hege_rover/model.sdf.
model_name="hege_rover"

for tool in xacro gz; do
  command -v "${tool}" >/dev/null 2>&1 || {
    echo "error: ${tool} not found. Source /opt/ros/humble/setup.bash and make sure gz-harmonic is installed." >&2
    exit 1
  }
done

[ -f "${xacro_file}" ] || { echo "error: ${xacro_file} not found" >&2; exit 1; }

echo "1/3 expanding xacro (drive:=px4)"
xacro "${xacro_file}" drive:=px4 -o "${urdf_out}"

echo "2/3 converting URDF to SDF"
mkdir -p "${model_dir}"
# `gz sdf -p` writes the converted model to stdout and its complaints to
# stderr. Keep stderr visible: sdformat reports a dropped sensor or an
# unparsed <gazebo> block as a warning, not as a failure, and a silently
# dropped imu_sensor is a vehicle whose EKF2 never starts.
gz sdf -p "${urdf_out}" > "${model_dir}/model.sdf"

# xacro names the robot "hege"; PX4 needs the directory and the model to agree.
sed -i -E "0,/<model name=('|\")[^'\"]*('|\")>/s//<model name=\"${model_name}\">/" \
  "${model_dir}/model.sdf"

echo "3/3 checking the four names PX4 hardcodes"
missing=0
for needle in 'name="base_link"' 'imu_sensor' 'navsat_sensor' 'magnetometer_sensor' 'air_pressure_sensor'; do
  if ! grep -q "${needle}" "${model_dir}/model.sdf"; then
    echo "  MISSING: ${needle}" >&2
    missing=1
  fi
done

if [ "${missing}" -ne 0 ]; then
  cat >&2 <<'EOF'

The generated model is missing something PX4 subscribes to by a fixed topic
path (see GZBridge.cpp). PX4 will boot, Gazebo will show the rover, and EKF2
will never get that sensor. Most likely causes:

  - base_link was lumped into base_footprint, meaning the preserveFixedJoint
    on base_joint in the px4 branch of hege.urdf.xacro did not take effect;
  - sdformat dropped a <gazebo reference="base_link"> sensor block, which it
    reports as a warning above rather than as an error.
EOF
  exit 1
fi

echo
echo "wrote ${model_dir}/model.sdf"
echo "next: scripts/install_to_px4.sh <path-to-PX4-Autopilot>"
