#!/usr/bin/env bash
#
# Put the HEGE model and airframe into a PX4-Autopilot checkout.
#
#     scripts/install_to_px4.sh ~/PX4-Autopilot
#
# PX4 resolves a SITL model as ${PX4_GZ_MODELS}/<name>/model.sdf, and
# PX4_GZ_MODELS is one directory rather than a search path, so the model has to
# live inside the PX4 tree. This symlinks ours in rather than copying, so
# regenerating the model takes effect without reinstalling.
#
# Everything this touches is untracked or a one-line addition, and --uninstall
# reverses all of it. Nothing in PX4 is modified in place.

set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg="$(dirname "${here}")"

airframe="4100_gz_hege_rover"
model_name="hege_rover"
anchor="4012_gz_rover_ackermann"   # the stock rover, which we register next to

uninstall=0
if [ "${1:-}" = "--uninstall" ]; then uninstall=1; shift; fi

px4_dir="${1:-${PX4_DIR:-}}"
if [ -z "${px4_dir}" ]; then
  echo "usage: $(basename "$0") [--uninstall] <path-to-PX4-Autopilot>" >&2
  exit 1
fi
px4_dir="$(cd "${px4_dir}" && pwd)"

airframe_dir="${px4_dir}/ROMFS/px4fmu_common/init.d-posix/airframes"
models_dir="${px4_dir}/Tools/simulation/gz/models"
cmakelists="${airframe_dir}/CMakeLists.txt"

for path in "${airframe_dir}" "${models_dir}" "${cmakelists}"; do
  [ -e "${path}" ] || {
    echo "error: ${path} not found. Is ${px4_dir} a PX4-Autopilot checkout with submodules initialised?" >&2
    echo "       Tools/simulation/gz is a submodule: git submodule update --init --recursive" >&2
    exit 1
  }
done

if [ "${uninstall}" -eq 1 ]; then
  rm -f "${airframe_dir}/${airframe}" "${models_dir}/${model_name}"
  sed -i "/^[[:space:]]*${airframe}[[:space:]]*$/d" "${cmakelists}"
  echo "removed ${airframe} and ${model_name} from ${px4_dir}"
  echo "run 'make clean' in PX4 before the next build"
  exit 0
fi

if [ ! -f "${pkg}/models/${model_name}/model.sdf" ]; then
  echo "error: models/${model_name}/model.sdf does not exist yet." >&2
  echo "       It is generated, not committed. Run scripts/generate_hege_model.sh first." >&2
  exit 1
fi

ln -sfn "${pkg}/models/${model_name}" "${models_dir}/${model_name}"
echo "linked ${models_dir}/${model_name}"

install -m 0755 "${pkg}/airframes/${airframe}" "${airframe_dir}/${airframe}"
echo "installed ${airframe_dir}/${airframe}"

# px4_add_romfs_files() is a plain list; an unregistered airframe file is
# simply never built into the ROMFS and `make px4_sitl gz_hege_rover` then
# fails with an unknown target.
if grep -q "^[[:space:]]*${airframe}[[:space:]]*$" "${cmakelists}"; then
  echo "already registered in $(basename "${cmakelists}")"
else
  sed -i "/^\([[:space:]]*\)${anchor}[[:space:]]*$/a\\\t${airframe}" "${cmakelists}"
  grep -q "^[[:space:]]*${airframe}[[:space:]]*$" "${cmakelists}" || {
    echo "error: could not register ${airframe}; add it to ${cmakelists} by hand next to ${anchor}" >&2
    exit 1
  }
  echo "registered in $(basename "${cmakelists}")"
fi

cat <<EOF

Done. Build and run:

    cd ${px4_dir}
    make px4_sitl gz_hege_rover

The first build after registering a new airframe has to regenerate the ROMFS,
so it takes longer than an incremental one.
EOF
