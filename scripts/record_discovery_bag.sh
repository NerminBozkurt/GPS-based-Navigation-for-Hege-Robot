#!/usr/bin/env bash
set -euo pipefail

label="${1:-00_discovery_all}"
bag_root="${2:-data/bags}"
output_path="${bag_root}/${label}_$(date -u +%Y%m%dT%H%M%SZ)"

mkdir -p "${bag_root}"
echo "Recording every discovered ROS topic to ${output_path}"
echo "Stop safely with Ctrl+C after 30–60 seconds."
exec ros2 bag record -a -s mcap -o "${output_path}"
