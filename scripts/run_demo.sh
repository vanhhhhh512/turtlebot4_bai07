#!/usr/bin/env bash
set -eo pipefail
package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd "$package_dir/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -n "${BAI07_UNDERLAY:-}" ]]; then source "$BAI07_UNDERLAY/install/setup.bash"; fi
cd "$workspace_dir"
if [[ "${BAI07_SKIP_BUILD:-0}" != 1 ]]; then colcon build --symlink-install --packages-select bai07_turtlebot4; fi
source "$workspace_dir/install/setup.bash"
export ROS_DOMAIN_ID="${BAI07_ROS_DOMAIN_ID:-97}"
exec 9>"/tmp/bai07_ros_domain_${ROS_DOMAIN_ID}.lock"
flock -n 9 || { printf "Bài 07 đang dùng domain %s; chọn BAI07_ROS_DOMAIN_ID khác.\n" "$ROS_DOMAIN_ID" >&2; exit 1; }
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="${BAI07_IGN_PARTITION:-bai07_$(date +%Y%m%d_%H%M%S)_$$}"
: "${DISPLAY:?X11 DISPLAY is required for the demo}"
run_dir="${BAI07_OUTPUT_DIR:-$HOME/bai07_runs/$(date +%Y%m%d_%H%M%S)_$$}"
mkdir -p "$run_dir"
printf 'export ROS_DOMAIN_ID=%q\nexport ROS_LOCALHOST_ONLY=1\nexport IGN_PARTITION=%q\nexport BAI07_RUN=%q\n' "$ROS_DOMAIN_ID" "$IGN_PARTITION" "$run_dir" > "$run_dir/session.env"
python3 "$package_dir/scripts/arrange_windows.py" > "$run_dir/windows.log" 2>&1 &
arranger_pid=$!
trap 'kill "$arranger_pid" 2>/dev/null || true' EXIT
printf 'Run: %s\nDomain: %s, partition: %s\n' "$run_dir" "$ROS_DOMAIN_ID" "$IGN_PARTITION"
ros2 launch bai07_turtlebot4 demo.launch.py output_dir:="$run_dir" "$@" 2>&1 | tee "$run_dir/launch.log"
