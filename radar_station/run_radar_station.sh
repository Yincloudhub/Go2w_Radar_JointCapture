#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${EUID}" -ne 0 ]]; then
  sudo_env=()
  while IFS='=' read -r env_name env_value; do
    case "${env_name}" in
      RADAR_*|FPGA_UDP_OVERRIDE_*)
        sudo_env+=("${env_name}=${env_value}")
        ;;
    esac
  done < <(env)

  if [[ -n "${RADAR_STATION_SUDO_PASSWORD:-}" ]]; then
    printf "%s\n" "${RADAR_STATION_SUDO_PASSWORD}" | sudo -S -p "" env "${sudo_env[@]}" "$0" "$@"
    exit $?
  fi
  if sudo -n "$0" show-config >/dev/null 2>&1; then
    exec sudo env "${sudo_env[@]}" "$0" "$@"
  fi
  if [[ ! -t 0 ]]; then
    echo "Passwordless sudo is required for non-interactive station control. Configure sudoers or provide RADAR_STATION_SUDO_PASSWORD." >&2
    exit 2
  fi
  exec sudo env "${sudo_env[@]}" "$0" "$@"
fi

resolve_fpga_udp_override() {
  if [[ -n "${FPGA_UDP_OVERRIDE_DIR:-}" ]]; then
    return 0
  fi

  local build_dir native_dir
  for build_dir in \
    "${ROOT_DIR}/fpga_udp/build"
  do
    [[ -d "${build_dir}" ]] || continue
    for native_dir in "${build_dir}"/*; do
      [[ -d "${native_dir}" ]] || continue
      if compgen -G "${native_dir}/fpga_udp*.so" > /dev/null || compgen -G "${native_dir}/fpga_udp*.pyd" > /dev/null; then
        export FPGA_UDP_OVERRIDE_DIR="${native_dir}"
        return 0
      fi
    done
  done
}

case "${RADAR_READER_MODE:-}" in
  framefile2|framefile|file)
    resolve_fpga_udp_override
    ;;
esac
export PYTHONUNBUFFERED=1
if [[ -z "${RADAR_READY_SOCKET:-}" && -S /run/radar-ready.sock ]]; then
  export RADAR_READY_SOCKET=/run/radar-ready.sock
fi
if [[ "${1:-}" != "show-config" && "${1:-}" != "ready-status" ]]; then
  "${ROOT_DIR}/tools/wait_for_clock.sh"
fi
cd "${ROOT_DIR}"
exec python3 "${ROOT_DIR}/radar_station_control.py" "$@"
