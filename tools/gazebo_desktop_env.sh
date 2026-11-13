#!/usr/bin/env bash
# Source only: discover this user's desktop auth, never disable X security.
while read -r pid; do
  [[ -r /proc/$pid/cmdline ]] || continue
  mapfile -d '' -t xargs < "/proc/$pid/cmdline"
  xdisplay='' xauth=''
  for ((j=0; j<${#xargs[@]}; j++)); do
    [[ "${xargs[j]}" == :* ]] && xdisplay="${xargs[j]}"
    [[ "${xargs[j]}" == -auth ]] && xauth="${xargs[j+1]}"
  done
  if [[ -n "$xdisplay" && ( -z "${DISPLAY:-}" || "$DISPLAY" == "$xdisplay" ) ]]; then
    export DISPLAY="$xdisplay"
    [[ -n "${XAUTHORITY:-}" || ! -r "$xauth" ]] || export XAUTHORITY="$xauth"
    break
  fi
done < <(pgrep -u "$(id -u)" -x Xwayland || true)
if [[ -z "${DISPLAY:-}" ]]; then
  echo 'No desktop DISPLAY found. Run from your logged-in desktop terminal.' >&2
  return 2
fi
export QT_QPA_PLATFORM=xcb
