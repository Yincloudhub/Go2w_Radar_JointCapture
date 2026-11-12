#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
seed_file="${project_root}/data/clock/last_good_epoch"
min_epoch=1767225600 # 2026-01-01 UTC
max_epoch=4102444800 # 2100-01-01 UTC

rtc_epoch=$(cat /sys/class/rtc/rtc0/since_epoch 2>/dev/null || true)
now=$(date +%s)
if [[ "$rtc_epoch" =~ ^[0-9]+$ ]] &&
   (( rtc_epoch >= min_epoch && rtc_epoch < max_epoch && now + 5 < rtc_epoch )); then
  date -u -s "@$rtc_epoch" >/dev/null
  printf 'restored clock from backup-powered rtc0: %s\n' "$(date -Is)"
fi

if [[ -r "$seed_file" ]]; then
  seed=$(<"$seed_file")
  now=$(date +%s)
  if [[ "$seed" =~ ^[0-9]+$ ]] && (( seed >= min_epoch && seed < max_epoch && now < seed )); then
    date -u -s "@$seed" >/dev/null
    printf 'restored last known clock seed: %s\n' "$(date -Is)"
  fi
fi

for attempt in $(seq 1 24); do
  now=$(date +%s)
  if (( now >= min_epoch && now < max_epoch )) && chronyc waitsync 1 0.05 0 1 >/dev/null 2>&1; then
    printf 'clock synchronized by chrony at %s\n' "$(date -Is)"
    exit 0
  fi

  if (( attempt == 3 || attempt % 6 == 0 )); then
    epoch=$(runuser -u unitree -- env HOME=/home/unitree ssh -F /home/unitree/.ssh/config \
      -o BatchMode=yes -o ConnectTimeout=3 radar0 date +%s 2>/dev/null || true)
    if [[ "$epoch" =~ ^[0-9]+$ ]] && (( epoch >= min_epoch && epoch < max_epoch )); then
      date -u -s "@$epoch" >/dev/null
      chronyc online 192.168.55.1 >/dev/null 2>&1 || true
      chronyc burst 4/4 192.168.55.1 >/dev/null 2>&1 || true
      printf 'clock initialized from radar0 at %s; awaiting chrony refinement\n' "$(date -Is)"
      exit 0
    fi
  fi
  sleep 2
done

printf 'no trusted NTP or radar0 time source; capture must remain blocked\n' >&2
exit 1
