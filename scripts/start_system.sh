#!/usr/bin/env bash
# Master Laptop Startup Script (No Docker required)
# 1. Loads .env (GEMINI_API_KEY, ROS_DOMAIN_ID, ROBOT_IP, ROBOT_USER)
# 2. Starts TurtleBot3 robot bringup over SSH
# 3. Starts Laptop Brain (rosbridge + PID follower + Rehab Coach + Gemini 3.8 Live)
# 4. Starts native Node-RED on 0.0.0.0:1880 for Phone & Laptop browser access

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
    echo "[ERROR] Missing .env file! Copy .env.example to .env and add your GEMINI_API_KEY."
    exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-108}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
ROBOT_IP="${ROBOT_IP:-192.168.1.108}"
ROBOT_USER="${ROBOT_USER:-ubuntu}"
LAPTOP_IP="$(hostname -I | awk '{print $1}')"

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

cleanup() {
    echo ""
    echo "[TrainerBot] Shutting down Laptop Brain and Node-RED..."
    kill "${BRAIN_PID:-}" "${NODERED_PID:-}" "${ROBOT_SSH_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "============================================================"
echo " Starting TrainerBot Rehab System (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo "============================================================"

echo "[1/3] Starting TurtleBot3 Robot Hardware I/O over SSH ($ROBOT_USER@$ROBOT_IP)..."
ssh -t "${ROBOT_USER}@${ROBOT_IP}" \
    "source /opt/ros/jazzy/setup.bash && source ~/turtlebot3_ws/install/setup.bash && export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && export TURTLEBOT3_MODEL=burger && ros2 launch trainerbot_rehab robot_bringup.launch.py" &
ROBOT_SSH_PID=$!

sleep 6

echo "[2/3] Starting Laptop Brain (rosbridge + PID follower + Rehab Coach + Gemini Live)..."
ros2 launch trainerbot_rehab laptop_brain.launch.py &
BRAIN_PID=$!

sleep 3

echo "[3/3] Starting Native Node-RED (No Docker)..."
mkdir -p ~/.node-red
cp -f "$PKG_DIR/node_red/flows.json" ~/.node-red/flows.json
node-red -p 1880 -H 0.0.0.0 --userDir ~/.node-red &
NODERED_PID=$!

echo ""
echo "============================================================"
echo " ALL SYSTEMS RUNNING!"
echo " Open this URL on your PHONE or LAPTOP:"
echo "   http://${LAPTOP_IP}:1880/dashboard/page1"
echo "============================================================"

wait
