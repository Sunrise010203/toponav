#!/usr/bin/env bash
# Default: owned offscreen display. --gui: real desktop window, no Xvfb.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
source "$ROOT/tools/gazebo_env.sh"
bash "$ROOT/tools/build_gazebo_receiver.sh"
XVFB="$TOPONAV_GAZEBO_PREFIX/usr/bin/Xvfb"
if [[ ! -x "$XVFB" || ! -x "$TOPONAV_GAZEBO_PREFIX/usr/bin/gz" ]]; then
  echo 'Install first: bash tools/install_gazebo_local.sh' >&2
  exit 2
fi
export PYTHONDONTWRITEBYTECODE=1
for arg in "$@"; do
  if [[ "$arg" == --gui ]]; then
    source "$ROOT/tools/gazebo_desktop_env.sh"
    exec "$ROOT/.venv/bin/python" -m simulation.gazebo_cross_validation --render-mode display "$@"
  fi
done
TMP=$(mktemp -d)
XPID=''
cleanup() {
  if [[ -n "$XPID" ]]; then kill "$XPID" 2>/dev/null || true; wait "$XPID" 2>/dev/null || true; fi
  rm -rf "$TMP"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# Xvfb chooses an unused display atomically. Never disturb a user's X server.
"$XVFB" -displayfd 3 -screen 0 1280x960x24 -ac -nolisten tcp \
  3>"$TMP/display" >"$TMP/xvfb.log" 2>&1 &
XPID=$!
for ((i=0; i<100; i++)); do
  [[ -s "$TMP/display" ]] && break
  if ! kill -0 "$XPID" 2>/dev/null; then cat "$TMP/xvfb.log" >&2; exit 2; fi
  sleep .1
done
if [[ ! -s "$TMP/display" ]]; then echo 'Xvfb startup timeout' >&2; exit 2; fi
export DISPLAY=":$(cat "$TMP/display")"
"$ROOT/.venv/bin/python" -m simulation.gazebo_cross_validation --render-mode display "$@"
