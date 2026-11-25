#!/usr/bin/env bash
set -u

# Wait for a usable wall clock before a station is armed or captured.
min_epoch="${NX_MIN_VALID_EPOCH:-1704067200}"
max_wait="${NX_CLOCK_WAIT_SECONDS:-90}"
start_epoch="$(date +%s 2>/dev/null || printf '0')"
deadline=$((start_epoch + max_wait))

kick_chrony() {
    command -v chronyc >/dev/null 2>&1 || return 0
    chronyc online >/dev/null 2>&1 || true
    chronyc burst 4/4 >/dev/null 2>&1 || true
    chronyc makestep >/dev/null 2>&1 || true
}

kick_chrony
while :; do
    now="$(date +%s 2>/dev/null || printf '0')"
    if [[ "$now" =~ ^[0-9]+$ ]] && (( now >= min_epoch )); then
        if ! command -v chronyc >/dev/null 2>&1 || chronyc waitsync 1 0.05 0 1 >/dev/null 2>&1; then
            printf 'clock ready: %s\n' "$(date -Ins 2>/dev/null || date)" >&2
            exit 0
        fi
    fi
    if (( now >= deadline )); then
        printf 'clock not ready after %ss: epoch=%s; capture blocked\n' "$max_wait" "$now" >&2
        timedatectl status >&2 2>/dev/null || true
        exit 3
    fi
    if (( (now - start_epoch) % 10 < 2 )); then
        kick_chrony
    fi
    sleep 2
done
