#!/usr/bin/env bash
# Download verified Ubuntu/OSRF .deb dependencies and extract a private prefix.
# NEVER installs packages into the OS, changes ROS or invokes sudo.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 || "$(uname -m)" != x86_64 ]]; then
  echo 'This local binary setup is scoped to Ubuntu 22.04 amd64.' >&2; exit 2
fi
CACHE="$ROOT/.cache/gazebo-harmonic/apt"
PREFIX="$ROOT/.simulators/gazebo-harmonic"
mkdir -p "$CACHE/lists/partial" "$CACHE/archives/partial" "$CACHE/sources.list.d" "$PREFIX"
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o "$CACHE/gazebo.gpg"
printf 'deb [signed-by=%s/gazebo.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main\n' "$CACHE" > "$CACHE/sources.list"
cat > "$CACHE/apt.conf" <<EOF
Dir::Etc::sourcelist "$CACHE/sources.list";
Dir::Etc::sourceparts "$CACHE/sources.list.d";
Dir::State::lists "$CACHE/lists";
Dir::Cache::archives "$CACHE/archives";
Dir::Cache::pkgcache "$CACHE/pkgcache.bin";
Dir::Cache::srcpkgcache "$CACHE/srcpkgcache.bin";
Debug::NoLocking "true";
APT::Sandbox::User "$(id -un)";
EOF
# Refresh only the private OSRF index. Never update through system-index links.
apt-get -c "$CACHE/apt.conf" -o Dir::Etc::sourceparts=- update
for file in /var/lib/apt/lists/*; do
  [[ -f "$file" && ! -e "$CACHE/lists/$(basename "$file")" ]] && ln -s "$file" "$CACHE/lists/$(basename "$file")" || true
done
[[ ! -e "$CACHE/sources.list.d/ubuntu.list" ]] && ln -s /etc/apt/sources.list "$CACHE/sources.list.d/ubuntu.list" || true
for file in /etc/apt/sources.list.d/*.sources /etc/apt/sources.list.d/*.list; do
  [[ -f "$file" && ! -e "$CACHE/sources.list.d/$(basename "$file")" ]] && ln -s "$file" "$CACHE/sources.list.d/$(basename "$file")" || true
done
apt-get -c "$CACHE/apt.conf" --no-install-recommends --download-only -y install \
  gz-sim8-cli libgz-sim8-plugins libgz-sim8-dev python3-gz-transport13 python3-gz-msgs10 python3-pyqt5 xvfb
for file in "$CACHE/archives/"*.deb; do dpkg-deb -x "$file" "$PREFIX"; done
# Relocate only Gazebo's Ruby command descriptors in this private prefix.
sed -i "s|library_path: /usr/|library_path: $PREFIX/usr/|" "$PREFIX/usr/share/gz/"*.yaml
sha256sum "$CACHE/archives/"*.deb > "$PREFIX/DOWNLOADED_DEBS.sha256"
source "$ROOT/tools/gazebo_env.sh"
gz sim --versions
"$ROOT/.venv/bin/python" -c 'from gz.transport13 import Node; from gz.msgs10.image_pb2 import Image; print("Gazebo Python transport ready")'
echo "Local prefix ready: $PREFIX"
