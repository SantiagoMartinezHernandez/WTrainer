#!/usr/bin/env bash
# Master Laptop Startup Script (No Docker required)
# 1. Loads .env (GEMINI_API_KEY, ROS_DOMAIN_ID, ROBOT_IP, ROBOT_USER)
# 2. Starts TurtleBot3 robot bringup over SSH
# 3. Starts Laptop Brain (rosbridge + PID follower + Rehab Coach + Gemini 3.8 Live)
# 4. Starts native Node-RED on 0.0.0.0:1880 for Phone & Laptop browser access

set +u
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$(dirname "$(dirname "$PKG_DIR")")"

if [ -f "$PKG_DIR/.env" ]; then
    set -a
    # shellcheck disable=SC1091
    source "$PKG_DIR/.env"
    set +a
elif [ -f "$WS_DIR/.env" ]; then
    set -a
    # shellcheck disable=SC1091
    source "$WS_DIR/.env"
    set +a
else
    echo "[WARN] No .env file found; continuing in offline/local coaching mode (Gemini Live disabled)."
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-108}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
ROBOT_IP="${1:-${ROBOT_IP:-192.168.1.108}}"
ROBOT_USER="${ROBOT_USER:-ubuntu}"
LAPTOP_IP="$(hostname -I | awk '{print $1}')"

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
set +u
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

LAPTOP_PAT="laptop_brain.launch.py|rosbridge_websocket|rosapi_node|vision_pose_node|rehab_coach_node|gemini_live_node|speaker_node|node-red"
PI_PAT="robot_bringup|all_on_bot|turtlebot3_ros|single_coin_d4_node|ld08_driver|hlds_laser_publisher|robot_state_publisher|usb_cam_node_exe|robot_pid_controller_node|servo_node|mic_node|speaker_node|vision_pose_node|rehab_coach_node|gemini_live_node|rosbridge_websocket|rosapi_node|node-red"
PI_INT_CMD="pkill -INT -f '${PI_PAT}' 2>/dev/null || true"
PI_TERM_CMD="pkill -TERM -f '${PI_PAT}' 2>/dev/null || true"
PI_KILL_CMD="pkill -9 -f '${PI_PAT}' 2>/dev/null || true"

cleanup() {
    trap - EXIT INT TERM
    echo ""
    echo "[TrainerBot] Shutting down Laptop Brain, Node-RED, and TurtleBot3 Pi..."
    kill -TERM "${BRAIN_PID:-}" "${NODERED_PID:-}" "${ROBOT_SSH_PID:-}" 2>/dev/null || true
    pkill -TERM -f "$LAPTOP_PAT" 2>/dev/null || true
    if ssh -o BatchMode=yes -o ConnectTimeout=2 "${ROBOT_USER}@${ROBOT_IP}" true >/dev/null 2>&1; then
        ssh -o BatchMode=yes -o ConnectTimeout=5 "${ROBOT_USER}@${ROBOT_IP}" "${PI_INT_CMD}; sleep 1.5; ${PI_TERM_CMD}; sleep 1.0; ${PI_KILL_CMD}" >/dev/null 2>&1 || true
    else
        sleep 1.5
    fi
    pkill -9 -f "$LAPTOP_PAT" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "============================================================"
echo " Starting TrainerBot Rehab System (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo "============================================================"

echo "[1/3] Checking SSH to TurtleBot3 ($ROBOT_USER@$ROBOT_IP)..."
PI_CMD="set +u; export AMENT_TRACE_SETUP_FILES=\${AMENT_TRACE_SETUP_FILES:-}; LDS_LINE=\$(grep -E '^[[:space:]]*export[[:space:]]+LDS_MODEL=' ~/.bashrc 2>/dev/null | tail -n1); [ -n \"\$LDS_LINE\" ] && eval \"\$LDS_LINE\"; export LDS_MODEL=\${LDS_MODEL:-LDS-02}; source /opt/ros/jazzy/setup.bash && source ~/turtlebot3_ws/install/setup.bash && export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && export TURTLEBOT3_MODEL=burger && ros2 launch trainerbot_rehab robot_bringup.launch.py"
if ssh -o BatchMode=yes -o ConnectTimeout=3 "${ROBOT_USER}@${ROBOT_IP}" true >/dev/null 2>&1; then
    ssh -o BatchMode=yes -o ConnectTimeout=3 "${ROBOT_USER}@${ROBOT_IP}" "$PI_KILL_CMD" >/dev/null 2>&1 || true
    sleep 1
    ssh -t "${ROBOT_USER}@${ROBOT_IP}" "$PI_CMD" &
    ROBOT_SSH_PID=$!
    sleep 6
else
    echo "[WARN] Passwordless SSH key not set up yet for ${ROBOT_USER}@${ROBOT_IP}."
    echo "       Run once: ssh-copy-id ${ROBOT_USER}@${ROBOT_IP}"
    echo "       Or in a separate Pi SSH terminal, run:"
    echo "         ros2 launch trainerbot_rehab robot_bringup.launch.py"
fi

echo "[2/3] Starting Laptop Brain (rosbridge + PID follower + Rehab Coach + Gemini Live)..."
ros2 launch trainerbot_rehab laptop_brain.launch.py &
BRAIN_PID=$!

sleep 3

echo "[3/3] Starting Native Node-RED (No Docker)..."
NODE_RED_BIN="$(command -v /usr/local/bin/node-red || command -v /usr/bin/node-red || which -a node-red 2>/dev/null | grep -v '^/mnt/' | head -n1 || true)"
if [ -z "$NODE_RED_BIN" ]; then
    echo "[ERROR] Native Linux node-red not found (only Windows /mnt/c/... was in PATH)."
    echo "Run: sudo apt install -y nodejs npm && sudo npm install -g --unsafe-perm node-red"
    exit 1
fi
mkdir -p ~/.node-red
cp -f "$PKG_DIR/node_red/flows.json" ~/.node-red/flows.json
"$NODE_RED_BIN" -p 1880 --userDir ~/.node-red ~/.node-red/flows.json &
NODERED_PID=$!

echo ""
echo "============================================================"
echo " ALL SYSTEMS RUNNING!"
echo " Open on LAPTOP browser:"
echo "   http://127.0.0.1:1880/dashboard/page1"
echo " Open on PHONE browser (same Wi-Fi, use Windows Wi-Fi IP if WSL2):"
echo "   http://${LAPTOP_IP}:1880/dashboard/page1"
echo "============================================================"

wait
