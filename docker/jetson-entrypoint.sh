#!/bin/bash
set -euo pipefail
if [[ ! -r /run/host-nv_tegra_release ]]; then
  echo 'Mount host /etc/nv_tegra_release at /run/host-nv_tegra_release for L4T compatibility checking.' >&2
  exit 1
fi
image_version=$(cut -d- -f1 /opt/odin2/l4t-version)
host_version=$(sed -nE 's/^# R([0-9]+).*REVISION: ([0-9.]+),.*/\1.\2/p' /run/host-nv_tegra_release)
if [[ "$host_version" != "$image_version" ]]; then
  echo "Jetson L4T mismatch: host=$host_version image=$image_version; rebuild with a matching L4T_BASE_IMAGE." >&2
  exit 1
fi
exec /odin2-entrypoint.sh "$@"
