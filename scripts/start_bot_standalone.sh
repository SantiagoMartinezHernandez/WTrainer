#!/usr/bin/env bash
# Standalone All-on-Bot Startup Script (for 4 GB Raspberry Pi + Browser Node-RED)
# Launches ALL ROS 2 nodes + Node-RED directly on the TurtleBot3 Pi over SSH (or locally if run on Pi).
# Usage: bash scripts/start_bot_standalone.sh [ROBOT_IP]
set +u

ROBOT_IP="${1:-${ROBOT_IP:-192.168.1.108}}"
ROBOT_USER="${ROBOT_USER:-ubuntu}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-108}"

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
ros2 launch trainerbot_rehab all_on_bot.launch.py
wait \$NR_PID 2>/dev/null || true
"

IS_LOCAL=0
if [ "$ROBOT_IP" = "local" ] || [ "$ROBOT_IP" = "127.0.0.1" ] || hostname -I 2>/dev/null | grep -qw "$ROBOT_IP"; then
    IS_LOCAL=1
fi

echo "============================================================"
echo " TrainerBot Standalone All-on-Bot Mode (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo "============================================================"
echo " Open in ANY browser on the same Wi-Fi:"
echo "   http://${ROBOT_IP}:1880/dashboard/page1"
echo "============================================================"

if [ "$IS_LOCAL" -eq 1 ]; then
    eval "$PI_CLEANUP_CMD"
    bash -c "$REMOTE_RUN"
else
    ssh -o BatchMode=yes -o ConnectTimeout=3 "${ROBOT_USER}@${ROBOT_IP}" "$PI_CLEANUP_CMD" >/dev/null 2>&1 || true
    trap "ssh -o BatchMode=yes -o ConnectTimeout=3 '${ROBOT_USER}@${ROBOT_IP}' \"${PI_CLEANUP_CMD}\" >/dev/null 2>&1 || true" EXIT INT TERM
    ssh -t "${ROBOT_USER}@${ROBOT_IP}" "$REMOTE_RUN"
fi
