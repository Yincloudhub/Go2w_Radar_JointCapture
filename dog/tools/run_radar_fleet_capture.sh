#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ENV_FILE="${ROOT_DIR}/tools/radar_fleet_env.sh"
if [[ -f "${ENV_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
fi

export PYTHONUNBUFFERED=1
cd "${ROOT_DIR}"
exec python3 "${ROOT_DIR}/tools/radar_fleet_control.py" "$@"
