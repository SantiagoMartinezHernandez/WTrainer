#!/usr/bin/env bash
# Standalone All-on-Bot Startup Script (for 4 GB Raspberry Pi + Browser Node-RED)
# Launches ALL compute, vision, AI, and Node-RED on the TurtleBot3 Pi.
# When launched from your Laptop over SSH, borrows your Laptop's speakers by default
# until the MAX98357A amp arrives (pass `pi-speaker` as 2nd arg once Pi audio is wired).
#
# Usage:
#   bash scripts/start_bot_standalone.sh [ROBOT_IP] [laptop-speaker|pi-speaker]
set +u

ROBOT_IP="${1:-${ROBOT_IP:-192.168.1.108}}"
SPEAKER_ARG="${2:-}"
ROBOT_USER="${ROBOT_USER:-ubuntu}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-108}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$(dirname "$(dirname "$PKG_DIR")")"

IS_LOCAL=0
if [ "$ROBOT_IP" = "local" ] || [ "$ROBOT_IP" = "127.0.0.1" ] || hostname -I 2>/dev/null | grep -qw "$ROBOT_IP"; then
    IS_LOCAL=1
fi

USE_LAPTOP_SPEAKER=1
ENABLE_PI_SPEAKER="false"
if [ "$IS_LOCAL" -eq 1 ] || [ "$SPEAKER_ARG" = "pi-speaker" ] || [ "$SPEAKER_ARG" = "pi" ]; then
    USE_LAPTOP_SPEAKER=0
    ENABLE_PI_SPEAKER="true"
fi

PI_PAT="robot_bringup|all_on_bot|turtlebot3_ros|single_coin_d4_node|ld08_driver|hlds_laser_publisher|robot_state_publisher|usb_cam_node_exe|robot_pid_controller_node|servo_node|mic_node|speaker_node|vision_pose_node|rehab_coach_node|gemini_live_node|rosbridge_websocket|rosapi_node|node-red"
PI_CLEANUP_CMD="pkill -INT -f '${PI_PAT}' 2>/dev/null || true; sleep 1.5; pkill -TERM -f '${PI_PAT}' 2>/dev/null || true; sleep 1.0; pkill -9 -f '${PI_PAT}' 2>/dev/null || true"

REMOTE_RUN="
set +u
export AMENT_TRACE_SETUP_FILES=\${AMENT_TRACE_SETUP_FILES:-}
LDS_LINE=\$(grep -E '^[[:space:]]*export[[:space:]]+LDS_MODEL=' ~/.bashrc 2>/dev/null | tail -n1)
[ -n \"\$LDS_LINE\" ] && eval \"\$LDS_LINE\"
export LDS_MODEL=\${LDS_MODEL:-LDS-02}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

mkdir -p ~/.node-red
cp -f ~/turtlebot3_ws/src/trainerbot_rehab/node_red/flows.json ~/.node-red/flows.json
NODE_RED_BIN=\$(command -v node-red || true)
if [ -z \"\$NODE_RED_BIN\" ]; then
    echo '[ERROR] node-red not installed on Pi! Run: sudo apt install -y nodejs npm && sudo npm install -g --unsafe-perm node-red'
    exit 1
fi

trap '${PI_CLEANUP_CMD}' EXIT INT TERM
\"\$NODE_RED_BIN\" -p 1880 --userDir ~/.node-red ~/.node-red/flows.json &
NR_PID=\$!
sleep 2
ros2 launch trainerbot_rehab all_on_bot.launch.py enable_speaker:=${ENABLE_PI_SPEAKER}
wait \$NR_PID 2>/dev/null || true
"

echo "============================================================"
echo " TrainerBot Standalone All-on-Bot Mode (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
if [ "$USE_LAPTOP_SPEAKER" -eq 1 ]; then
    echo " Audio output: Borrowing Laptop speaker (laptop_speaker_node)"
else
    echo " Audio output: Pi onboard speaker (enable_speaker:=true)"
fi
echo " Open in ANY browser on the same Wi-Fi:"
echo "   http://${ROBOT_IP}:1880/dashboard/page1"
echo "============================================================"

LAPTOP_SPK_PID=""
cleanup() {
    trap - EXIT INT TERM
    echo ""
    echo "[TrainerBot] Stopping standalone Pi nodes, LiDAR, and audio..."
    if [ -n "${LAPTOP_SPK_PID:-}" ]; then
        kill -TERM "$LAPTOP_SPK_PID" 2>/dev/null || true
    fi
    pkill -TERM -f "laptop_speaker_node" 2>/dev/null || true
    if [ "$IS_LOCAL" -eq 1 ]; then
        eval "$PI_CLEANUP_CMD"
    else
        ssh -o BatchMode=yes -o ConnectTimeout=3 "${ROBOT_USER}@${ROBOT_IP}" "$PI_CLEANUP_CMD" >/dev/null 2>&1 || true
    fi
}

if [ "$IS_LOCAL" -eq 1 ]; then
    trap cleanup EXIT INT TERM
    eval "$PI_CLEANUP_CMD"
    bash -c "$REMOTE_RUN"
else
    trap cleanup EXIT INT TERM
    ssh -o BatchMode=yes -o ConnectTimeout=3 "${ROBOT_USER}@${ROBOT_IP}" "$PI_CLEANUP_CMD" >/dev/null 2>&1 || true

    if [ "$USE_LAPTOP_SPEAKER" -eq 1 ]; then
        export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
        export ROS_DOMAIN_ID
        source /opt/ros/jazzy/setup.bash
        source "$WS_DIR/install/setup.bash"
        ros2 run trainerbot_rehab speaker_node \
            --ros-args -r __node:=laptop_speaker_node \
            -p speaker_mode:=alsa \
            -p pcm_sample_rate:=24000 &
        LAPTOP_SPK_PID=$!
    fi

    ssh -t "${ROBOT_USER}@${ROBOT_IP}" "$REMOTE_RUN"
fi
