#!/usr/bin/env bash
# Build only inside this project. No sudo/system installation or network fetch.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
source tools/gazebo_env.sh
DEB="$ROOT/.cache/gazebo-harmonic/apt/archives/libgz-sim8-dev_8.15.0-1~jammy_amd64.deb"
EXPECTED=49b9186d1c6d4cba224cb533673649ad286e613f1018467927f21d1f52dc994a
[[ -f "$DEB" ]] || { echo 'Missing matched Gazebo 8.15.0 dev package; run tools/install_gazebo_local.sh first.' >&2; exit 2; }
[[ "$(sha256sum "$DEB" | cut -d' ' -f1)" == "$EXPECTED" ]] || { echo 'Gazebo dev package checksum mismatch' >&2; exit 2; }
OUT="$ROOT/.simulators/toponav-receiver"
mkdir -p "$OUT"
exec 9>"$OUT/build.lock"; flock 9
if "$ROOT/.venv/bin/python" -c 'from simulation.gazebo_receiver import verify_binary; verify_binary()' >/dev/null 2>&1; then
  echo "Verified receiver interlock: $OUT/libtoponav-receiver.so"
  exit 0
fi
DEV="$ROOT/.cache/gazebo-receiver-dev/$EXPECTED"
if [[ ! -f "$DEV/usr/include/gz/sim8/gz/sim/System.hh" ]]; then
  mkdir -p "$DEV"; dpkg-deb -x "$DEB" "$DEV"
fi
USR="$TOPONAV_GAZEBO_PREFIX/usr"
INCLUDES=(-I"$DEV/usr/include/gz/sim8" -I"$USR/include" -I/usr/include/eigen3)
for dir in "$USR"/include/gz/*; do [[ ! -d "$dir" ]] || INCLUDES+=(-I"$dir"); done
TMP=$(mktemp "$OUT/libtoponav-receiver.XXXXXX.so")
trap 'rm -f "$TMP"' EXIT
COMMAND=(g++ -std=c++17 -O2 -Wall -Wextra -Werror -fPIC -shared -pthread
  "${INCLUDES[@]}" simulation/native/gazebo_receiver_interlock.cc -o "$TMP"
  -L"$USR/lib/x86_64-linux-gnu" -Wl,-z,defs
  -l:libgz-sim8.so.8 -lgz-transport13 -lgz-plugin2 -lgz-msgs10 -lsdformat14 -lgz-math7 -lgz-common5 -l:libprotobuf.so.23)
"${COMMAND[@]}"
mv "$TMP" "$OUT/libtoponav-receiver.so"
"$ROOT/.venv/bin/python" - "$EXPECTED" "${COMMAND[@]}" <<'PY'
import hashlib,json,sys
from pathlib import Path
root=Path.cwd(); out=root/'.simulators/toponav-receiver'
names=['simulation/native/receiver_lease.hh','simulation/native/receiver_envelope.hh','simulation/native/receiver_status.hh','simulation/native/gazebo_receiver_interlock.cc','tools/build_gazebo_receiver.sh']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
metadata=dict(dev_package_sha256=sys.argv[1],command=sys.argv[2:],sources={n:sha(root/n) for n in names},binary_sha256=sha(out/'libtoponav-receiver.so'),gazebo_runtime_sha256=sha(root/'.simulators/gazebo-harmonic/usr/lib/x86_64-linux-gnu/libgz-sim8.so.8.15.0'))
(out/'build.json').write_text(json.dumps(metadata,indent=2)+'\n')
print('Built receiver interlock: '+str(out/'libtoponav-receiver.so'))
PY
