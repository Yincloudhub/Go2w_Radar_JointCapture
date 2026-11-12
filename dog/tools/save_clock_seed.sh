#!/usr/bin/env bash
set -euo pipefail

chronyc waitsync 1 0.05 0 1 >/dev/null 2>&1 || exit 0
epoch=$(date +%s)
(( epoch >= 1767225600 && epoch < 4102444800 )) || exit 0
hwclock --systohc --rtc=/dev/rtc0 --utc

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
directory="${project_root}/data/clock"
install -d -m 750 -o root -g unitree "$directory"
temporary=$(mktemp "$directory/.last_good_epoch.XXXXXX")
trap 'rm -f "$temporary"' EXIT
printf '%s\n' "$epoch" >"$temporary"
chgrp unitree "$temporary"
chmod 640 "$temporary"
mv -f "$temporary" "$directory/last_good_epoch"
