#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
source "$ROOT/tools/gazebo_env.sh"
source "$ROOT/tools/gazebo_desktop_env.sh"
export PYTHONDONTWRITEBYTECODE=1
exec "$ROOT/.venv/bin/python" -m tools.gazebo_scenarios_app "$@"
