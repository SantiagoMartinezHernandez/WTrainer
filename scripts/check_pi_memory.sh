#!/usr/bin/env bash
# Check live RAM / swap and top processes on the TurtleBot3 Raspberry Pi
# Usage: bash scripts/check_pi_memory.sh [ROBOT_IP]
set +u

ROBOT_IP="${1:-${ROBOT_IP:-192.168.1.108}}"
ROBOT_USER="${ROBOT_USER:-ubuntu}"

REMOTE_CMD='
echo "=== Pi Board ==="
tr -d "\0" </proc/device-tree/model 2>/dev/null || uname -m
echo ""
echo ""
echo "=== Memory & Swap (free -h) ==="
free -h
echo ""
echo "=== Top 15 Processes by RSS (KB) ==="
ps -eo pid,comm,%mem,rss --sort=-rss | head -n 16
echo ""
AVAIL_MB=$(awk "/MemAvailable/ {printf \"%d\", \$2/1024}" /proc/meminfo)
echo "=== Verdict ==="
echo "Available RAM: ${AVAIL_MB} MB"
echo "Reference extra load: MediaPipe (~200-250 MB) + Node-RED (~100 MB) + Google/Vosk (~120 MB)."
if [ "$AVAIL_MB" -lt 350 ]; then
    echo "-> Tight for heavy MediaPipe/Node-RED directly on Pi; keeping Perception/UI on Laptop prevents OOM kills."
else
    echo "-> Sufficient headroom for current on-robot stack (PID + LiDAR + Servo + Camera + Mic)."
fi
'

echo "Checking Pi memory on ${ROBOT_USER}@${ROBOT_IP}..."
ssh -o ConnectTimeout=5 "${ROBOT_USER}@${ROBOT_IP}" "$REMOTE_CMD"
