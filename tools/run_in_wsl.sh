#!/usr/bin/env bash
# Build and start the warehouse robot inside WSL (Ubuntu 22.04 + ROS 2 Humble).
# run_warehouse.bat calls this; you can also run it yourself from a WSL terminal:
#
#     bash tools/run_in_wsl.sh [--direct] [--build-only] [launch arguments...]
#
#   --direct      serve the order page on 127.0.0.1:8088, reached from Windows through WSL's
#                 own localhost forwarding (used when Python for Windows is not installed).
#                 Without it, the page is served on 127.0.0.2:8089 for tools/web_relay_windows.py.
#   --build-only  copy and build, then stop.
#   launch arguments are passed to bringup.launch.py, e.g. mode:=audit gui:=false
#
# The repository is copied into a colcon workspace on the Linux file system (fast builds),
# default ~/warehouse_ws; set WAREHOUSE_WS to use another one. The copy is only replaced if this
# script created it, so a clone you made there yourself is never overwritten.
#
# Machine-specific settings (DDS profile, Gazebo address, ...) can go in ~/warehouse_env.sh.
# It is sourced if it exists, and may define a function `warehouse_cleanup` that is run
# before the launch and after it ends.

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${WAREHOUSE_WS:-$HOME/warehouse_ws}"
DEST="$WS/src/warehouse_inventory_robot"
MARKER=".copied_by_run_in_wsl"
WEB_HOST=127.0.0.2
WEB_PORT=8089
BUILD_ONLY=0

die() { echo "!! $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  case "$1" in
    --direct) WEB_HOST=127.0.0.1; WEB_PORT=8088; shift ;;
    --build-only) BUILD_ONLY=1; shift ;;
    *) break ;;
  esac
done

[ -f /opt/ros/humble/setup.bash ] || die "ROS 2 Humble is not installed in this distribution (see README section 3)."
source /opt/ros/humble/setup.bash
[ -f /usr/share/gazebo/setup.sh ] && source /usr/share/gazebo/setup.sh
[ -f "$HOME/warehouse_env.sh" ] && source "$HOME/warehouse_env.sh"

# ---- copy the repository into the workspace -------------------------------------------------
if [ "$(readlink -f "$REPO")" = "$(readlink -f "$DEST")" ]; then
  echo ">> Building in place: $DEST"
else
  if [ -e "$DEST" ] && [ ! -f "$DEST/$MARKER" ]; then
    die "$DEST already exists and was not created by this script. Move it away, or set WAREHOUSE_WS to another workspace."
  fi
  echo ">> Copying the project into $DEST"
  STAGE="$WS/.copy_in_progress"
  rm -rf "$STAGE" && mkdir -p "$STAGE" "$WS/src" || die "cannot write to $WS"
  # tar keeps file times, so unchanged packages are not rebuilt
  tar -C "$REPO" --exclude=.git --exclude=__pycache__ --exclude='*.pyc' --exclude='*.zip' \
      --exclude=./build --exclude=./install --exclude=./log -cf - . | tar -C "$STAGE" -xf - \
    || die "copy failed"
  touch "$STAGE/$MARKER"
  rm -rf "$DEST" && mv "$STAGE" "$DEST" || die "cannot replace $DEST"
fi

# ---- build ------------------------------------------------------------------------------------
echo ">> Building (the first build takes a few minutes)..."
mkdir -p "$WS/log"
if ! (cd "$WS" && colcon build --symlink-install > "$WS/log/run_in_wsl_build.txt" 2>&1); then
  tail -n 25 "$WS/log/run_in_wsl_build.txt"
  die "Build failed. Full output: $WS/log/run_in_wsl_build.txt"
fi
tail -n 1 "$WS/log/run_in_wsl_build.txt"
source "$WS/install/setup.bash"
[ "$BUILD_ONLY" = 1 ] && { echo ">> Build only - done."; exit 0; }

# ---- launch -----------------------------------------------------------------------------------
cleanup() {
  if declare -F warehouse_cleanup >/dev/null; then
    warehouse_cleanup
  else
    pkill -f '[g]zserver.*warehouse\.world' 2>/dev/null   # a Gazebo left over from an earlier run
  fi
  return 0
}
cleanup
trap cleanup EXIT

if [ -z "$WAYLAND_DISPLAY" ] && [ -z "$DISPLAY" ]; then
  echo ">> No display (WSLg) found - running without the Gazebo and RViz windows."
  if command -v Xvfb >/dev/null; then
    pgrep -x Xvfb >/dev/null || (Xvfb :99 -screen 0 1280x720x24 >/dev/null 2>&1 &)
    export DISPLAY=:99      # the cameras still need an X server to render
  fi
  set -- gui:=false rviz:=false "$@"
fi

echo ">> Starting Gazebo, Nav2 and the robot nodes. Stop with Ctrl+C."
if [ "$WEB_HOST" = 127.0.0.2 ]; then
  echo ">> Order page: http://localhost:8088 in Windows (through the relay that run_warehouse.bat starts),"
  echo ">>             or http://127.0.0.2:8089 from inside Linux"
else
  echo ">> Order page: http://localhost:8088"
fi
echo ">> Results: ~/warehouse_results"
# start_mission / mission_delay only matter in audit mode (mode:=audit)
ros2 launch warehouse_bot bringup.launch.py web_host:="$WEB_HOST" web_port:="$WEB_PORT" \
    start_mission:=true mission_delay:=45.0 "$@"
