#!/usr/bin/env bash
set -euo pipefail

output_dir="${1:-data/inventory/$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "${output_dir}"

date --iso-8601=seconds | tee "${output_dir}/session_time.txt"
uname -a | tee "${output_dir}/uname.txt"
ip -br address | tee "${output_dir}/ip_address.txt"
lsusb | tee "${output_dir}/lsusb.txt"
ros2 node list | tee "${output_dir}/nodes.txt"
ros2 topic list -t | tee "${output_dir}/topics_and_types.txt"
ros2 service list -t | tee "${output_dir}/services_and_types.txt"
ros2 action list -t | tee "${output_dir}/actions_and_types.txt"
ros2 doctor --report | tee "${output_dir}/ros2_doctor.txt"

echo "Inventory saved in ${output_dir}"
echo "Run 'ros2 run tf2_tools view_frames' separately while TF is active."
