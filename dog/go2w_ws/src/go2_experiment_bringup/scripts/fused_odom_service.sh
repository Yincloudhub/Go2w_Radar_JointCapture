#!/usr/bin/env bash

# ============================================================
# Unitree Go2 + XT16 融合里程计服务管理脚本
#
# 使用：
#   ./fused_odom_service.sh start
#   ./fused_odom_service.sh stop
#
# 功能：
# start:
#   1.启动或获取 unitree_slam
#   2.启动或获取 xt16_driver
#   3.通过keyDemo启动Mapping
#   4.等待 /unitree/slam_mapping/odom
#   5.验证Odometry数据
#
# stop:
#   根据PID文件关闭服务
#
# ============================================================

set -e

BIN_DIR="/unitree/module/unitree_slam/bin"
NET_IF="eth0"

ODOM_TOPIC="/unitree/slam_mapping/odom"
MAPPING_KEY="${MAPPING_KEY:-q}"

LOG_DIR="${NX_FLEET_ROOT:-$HOME/NX_radar_fleet}/logs/go2w"

PID_DIR="$LOG_DIR"
SLAM_PID_FILE="$PID_DIR/unitree_slam.pid"
XT16_PID_FILE="$PID_DIR/xt16_driver.pid"

mkdir -p "$LOG_DIR"


log()
{
    echo "[fused_odom_service] $*"
}


die()
{
    echo "[fused_odom_service][ERROR] $*" >&2
    exit 1
}


save_pid()
{
    local name=$1
    local pid=$2
    local file=$3

    echo "$pid" > "$file"

    log "$name PID=$pid"
}


stop_service()
{
    local name=$1
    local file=$2

    if [ ! -f "$file" ]; then
        log "没有找到$name PID文件"
        return
    fi

    local pid
    pid=$(cat "$file")

    if kill -0 "$pid" 2>/dev/null; then
        log "停止$name，PID=$pid"

        kill "$pid" 2>/dev/null || true

        sleep 1

        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    else
        log "$name进程不存在"
    fi

    rm -f "$file"
}


if [ "$1" = "stop" ]; then

    stop_service "xt16_driver" "$XT16_PID_FILE"
    stop_service "unitree_slam" "$SLAM_PID_FILE"

    log "停止完成"
    exit 0
fi


if [ "$1" != "start" ]; then

    echo "使用方法:"
    echo "  $0 start"
    echo "  $0 stop"

    exit 1
fi


# ============================================================
# 启动模式
# ============================================================

[ -x "$BIN_DIR/unitree_slam" ] || die "找不到unitree_slam"
[ -x "$BIN_DIR/xt16_driver" ] || die "找不到xt16_driver"
[ -x "$BIN_DIR/keyDemo" ] || die "找不到keyDemo"


# ---------------- unitree_slam ----------------

SLAM_PID=$(pgrep -f "$BIN_DIR/unitree_slam" | head -n1 || true)

if [ -n "$SLAM_PID" ]; then

    log "unitree_slam已经运行"

else

    log "启动unitree_slam"

    nohup "$BIN_DIR/unitree_slam" \
        >"$LOG_DIR/unitree_slam.log" 2>&1 &

    SLAM_PID=$!

fi

save_pid "unitree_slam" "$SLAM_PID" "$SLAM_PID_FILE"


sleep 3


# ---------------- XT16 ----------------

XT16_PID=$(pgrep -f "$BIN_DIR/xt16_driver" | head -n1 || true)

if [ -n "$XT16_PID" ]; then

    log "xt16_driver已经运行"

else

    log "启动xt16_driver"

    nohup "$BIN_DIR/xt16_driver" \
        >"$LOG_DIR/xt16_driver.log" 2>&1 &

    XT16_PID=$!

fi

save_pid "xt16_driver" "$XT16_PID" "$XT16_PID_FILE"


sleep 3


# ---------------- Mapping ----------------

log "启动keyDemo Mapping"

python3 <<PY
import os
import pty
import time
import select
import signal

pid, fd = pty.fork()

if pid == 0:
    os.execv(
        "$BIN_DIR/keyDemo",
        ["$BIN_DIR/keyDemo", "$NET_IF"]
    )

time.sleep(2)

os.write(fd, b"$MAPPING_KEY")

start=time.time()
buffer=""

while time.time()-start < 10:

    ready,_,_=select.select([fd],[],[],0.5)

    if fd in ready:
        data=os.read(fd,4096)
        text=data.decode(errors="ignore")
        print(text,end="")

        buffer += text

        if "Successfully started mapping" in buffer:
            print("\nMapping启动成功")
            break

os.kill(pid, signal.SIGTERM)

PY


# ---------------- ODOM检查 ----------------

log "等待 $ODOM_TOPIC"

for i in $(seq 1 25)
do

    COUNT=$(ros2 topic info "$ODOM_TOPIC" 2>/dev/null \
        | awk '/Publisher count/{print $3}')

    if [ "$COUNT" -ge 1 ] 2>/dev/null
    then
        log "检测到ODOM Publisher"
        break
    fi

    sleep 1

done


timeout 3 ros2 topic echo "$ODOM_TOPIC" \
    >/tmp/odom_check.txt 2>&1 || true


if grep -q "position:" /tmp/odom_check.txt
then

    log "Odometry数据正常"

else

    die "没有收到Odometry数据"

fi


log "融合里程计初始化成功"
log "持续监测 unitree_slam 和 xt16_driver"

while true; do
    if ! kill -0 "$SLAM_PID" 2>/dev/null; then
        die "unitree_slam 已退出，要求重启 Go2w 服务"
    fi
    if ! kill -0 "$XT16_PID" 2>/dev/null; then
        die "xt16_driver 已退出，要求重启 Go2w 服务"
    fi
    sleep 5
done
