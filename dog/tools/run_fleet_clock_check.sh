#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export PYTHONUNBUFFERED=1
cd "${ROOT_DIR}"
exec python3 "${ROOT_DIR}/tools/fleet_clock_check.py" "$@"
