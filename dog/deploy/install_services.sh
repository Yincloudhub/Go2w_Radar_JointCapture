#!/usr/bin/env bash
set -euo pipefail

if [[ "$EUID" -ne 0 ]]; then
  exec sudo "$0" "$@"
fi

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dog_user="$(stat -c %U "$project_root")"
dog_home="$(getent passwd "$dog_user" | cut -d: -f6)"
if [[ -z "$dog_home" || ! -f "$project_root/go2w_ws/install/setup.bash" ]]; then
  echo "Build go2w_ws and verify the project owner before installing services." >&2
  exit 2
fi

install -d -o "$dog_user" -g "$dog_user" -m 755 "$project_root/logs/ros" "$project_root/data/sessions"
install -d -o "$dog_user" -g "$dog_user" -m 755 "$dog_home/bin"
ln -sfn "$project_root/tools/run_radar_fleet_ui.sh" "$dog_home/bin/nxui"
for name in Go2w_velocitySerial_Autostart.service Recovery_velocitySerialAutoStart.service \
            nx-clock-bootstrap.service nx-clock-seed.service nx-clock-seed.timer; do
  temporary="$(mktemp)"
  sed -e "s|@DOG_ROOT@|$project_root|g" \
      -e "s|@DOG_USER@|$dog_user|g" \
      -e "s|@DOG_HOME@|$dog_home|g" \
      "$project_root/deploy/$name.in" >"$temporary"
  install -m 644 "$temporary" "/etc/systemd/system/$name"
  rm -f "$temporary"
done

systemctl daemon-reload
systemctl enable nx-clock-bootstrap.service nx-clock-seed.timer Recovery_velocitySerialAutoStart.service
systemctl disable Go2w_velocitySerial_Autostart.service
if systemctl list-unit-files joint-sensor-runtime.service --no-legend 2>/dev/null | grep -q joint-sensor-runtime; then
  systemctl disable --now joint-sensor-runtime.service
fi
echo "Services installed. Recovery starts at boot; long-press X starts Go2w."
