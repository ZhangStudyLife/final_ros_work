#!/usr/bin/env bash

missing=0

check_cmd() {
  local name="$1"
  if command -v "$name" >/dev/null 2>&1; then
    printf "[OK] command %-18s %s\n" "$name" "$(command -v "$name")"
  else
    printf "[MISS] command %-15s\n" "$name"
    missing=1
  fi
}

check_rospack() {
  local name="$1"
  if rospack find "$name" >/dev/null 2>&1; then
    printf "[OK] rospack %-18s %s\n" "$name" "$(rospack find "$name")"
  else
    printf "[MISS] rospack %-15s\n" "$name"
    missing=1
  fi
}

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
else
  echo "[MISS] /opt/ros/noetic/setup.bash"
  exit 1
fi

set -u

if [ -f "$HOME/final_work/tb3_course_ws/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$HOME/final_work/tb3_course_ws/devel/setup.bash"
fi

echo "ROS_DISTRO=$(rosversion -d 2>/dev/null || true)"
echo "DISPLAY=${DISPLAY:-}"
echo "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-}"
echo "GAZEBO_IP=${GAZEBO_IP:-}"

check_cmd roscore
check_cmd roslaunch
check_cmd catkin_make
check_cmd gazebo
check_cmd rviz

check_rospack tb3_sim_course
check_rospack turtlebot3_gazebo
check_rospack turtlebot3_slam
check_rospack turtlebot3_navigation
check_rospack turtlebot3_teleop
check_rospack cv_bridge
check_rospack map_server
check_rospack gmapping

python3 - <<'PY'
import cv2
print("[OK] cv2", cv2.__version__, "aruco=", hasattr(cv2, "aruco"))
if not hasattr(cv2, "aruco"):
    raise SystemExit(1)
PY
if [ $? -ne 0 ]; then
  missing=1
fi

if [ "$missing" -eq 0 ]; then
  echo "Environment check passed."
else
  echo "Environment check failed. Install the missing packages listed in README.md."
fi

exit "$missing"
