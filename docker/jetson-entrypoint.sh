#!/bin/bash
set -euo pipefail
python3 /opt/odin2/jetson/jetson-support.py check
exec /odin2-entrypoint.sh "$@"
