#!/usr/bin/env bash
# Clean Shutdown Script for TrainerBot Rehab (stops all Laptop & Pi ROS 2 nodes, LiDAR motor & Node-RED)
# Usage: bash scripts/stop_system.sh [ROBOT_IP]
set +u

ROBOT_IP="${1:-${ROBOT_IP:-192.168.1.10}}"
ROBOT_USER="${ROBOT_USER:-ubuntu}"

LAPTOP_PAT="laptop_brain.launch.py|rosbridge_websocket|rosapi_node|vision_pose_node|rehab_coach_node|gemini_live_node|speaker_node|node-red"
PI_PAT="robot_bringup|all_on_bot|turtlebot3_ros|single_coin_d4_node|ld08_driver|hlds_laser_publisher|robot_state_publisher|usb_cam_node_exe|robot_pid_controller_node|servo_node|mic_node|speaker_node|vision_pose_node|rehab_coach_node|gemini_live_node|rosbridge_websocket|rosapi_node|node-red"
PI_STOP_CMD="pkill -INT -f '${PI_PAT}' 2>/dev/null || true; sleep 1.5; pkill -TERM -f '${PI_PAT}' 2>/dev/null || true; sleep 1.0; pkill -9 -f '${PI_PAT}' 2>/dev/null || true"

echo "[TrainerBot] Stopping Laptop Brain and Node-RED..."
pkill -INT -f "$LAPTOP_PAT" 2>/dev/null || true
sleep 0.5
pkill -TERM -f "$LAPTOP_PAT" 2>/dev/null || true

if ssh -o BatchMode=yes -o ConnectTimeout=3 "${ROBOT_USER}@${ROBOT_IP}" true >/dev/null 2>&1; then
    echo "[TrainerBot] Gracefully stopping TurtleBot3 Pi nodes & LiDAR motor (${ROBOT_USER}@${ROBOT_IP})..."
    ssh -o BatchMode=yes -o ConnectTimeout=5 "${ROBOT_USER}@${ROBOT_IP}" "$PI_STOP_CMD" >/dev/null 2>&1 || true
    echo "[TrainerBot] TurtleBot3 Pi nodes and LiDAR stopped."
else
    echo "[TrainerBot] Could not reach Pi via passwordless SSH (${ROBOT_USER}@${ROBOT_IP}); skipped Pi shutdown."
    sleep 1.5
fi

pkill -9 -f "$LAPTOP_PAT" 2>/dev/null || true
echo "[TrainerBot] All processes stopped cleanly."
