# WTrainer — Universal Human-Framing Rehabilitation TurtleBot3

**WTrainer** (`trainerbot_rehab`) is a safety-first, 3-device ROS 2 Jazzy rehabilitation personal trainer built on the TurtleBot3 Burger. It helps wheelchair users and individuals with lower-limb hemiplegia train upper-body and core muscles with live posture feedback, smooth automated camera framing, and real-time voice coaching via **Gemini 3.8 Live (`gemini-3.8-live`)**.

---

## 1. 3-Device Architecture

| Device | Role | ROS 2 Nodes |
| :--- | :--- | :--- |
| **TurtleBot3 Raspberry Pi** *(1 GB RAM)* | Hardware I/O, local LiDAR safety & PID motion | `robot.launch.py`, `usb_cam`, `robot_pid_controller_node` (20 Hz PID + 0.6 s connection-loss watchdog + 0.35 m collision stops), `servo_node` (50 Hz smooth tilt), `mic_node`, `speaker_node` |
| **Laptop** *(Ubuntu 24.04 / WSL2)* | Heavy vision, AI coaching & dashboard host | `vision_pose_node` (MediaPipe Pose), `rehab_coach_node` (rehab-first rep & posture loop), `gemini_live_node` (`gemini-3.8-live`), `rosbridge_websocket`, native **Node-RED** (no Docker) |
| **Phone** | Patient / therapist control panel | Mobile browser → `http://<LAPTOP_IP>:1880/dashboard/page1` |

> **Privacy Guarantee:** Only anonymized MediaPipe numeric joint angles, torso tilt, and exercise names are forwarded to Gemini — **never** raw video frames, images, or personal identifiers.

---

## 2. Physical Wiring (Raspberry Pi 40-Pin Header)

Hold the Raspberry Pi with the **USB/Ethernet ports pointing DOWN**. **Pin 1** is top-left; odd pins are on the left column, even pins on the right column.

| Component | Module Pin | Pi Physical Pin | BCM / Interface | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **INMP441 Mic** | `VDD` | Pin 1 | `3.3V` | **3.3V only** (never 5V) |
| **INMP441 Mic** | `GND` | Pin 6 | `GND` | Common ground |
| **INMP441 Mic** | `L/R` | Pin 9 | `GND` | Selects Left channel |
| **INMP441 Mic** | `SCK` | Pin 12 | `GPIO 18` | Shared I2S Bit Clock |
| **INMP441 Mic** | `WS` | Pin 35 | `GPIO 19` | Shared I2S Frame Clock |
| **INMP441 Mic** | `SD` | Pin 38 | `GPIO 20` | I2S Data In |
| **Camera PWM Servo** *(or Dynamixel via USB/OpenCR)* | Signal | Pin 32 | `GPIO 12` | Hardware PWM0 (`VCC` → Pin 4 `5V`, `GND` → Pin 14) |
| **MAX98357A Amp** *(with APKLVSR 3W 4Ω Speaker)* | `VIN` / `GND` | Pin 2 (`5V`) / Pin 6 (`GND`) | `5V` / `GND` | Connects 2-wire speaker `+`/`-` safely |
| **MAX98357A Amp** | `BCLK` / `LRC` / `DIN` | Pin 12 / Pin 35 / Pin 40 | `GPIO 18` / `19` / `21` | Shares clocks with INMP441 |

> **Before MAX98357A arrives:** Leave the 2-wire speaker disconnected from the Pi. `laptop_brain.launch.py` plays all chimes, TTS cues, and Gemini voice responses directly on your **Laptop speakers** by default (`enable_laptop_speaker:=true`).

---

## 3. Step-by-Step Installation (Laptop & Robot)

### A. Laptop Setup (Ubuntu 24.04 / WSL2)

```bash
# 1. Clone into your ROS 2 workspace
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone https://github.com/SantiagoMartinezHernandez/WTrainer.git trainerbot_rehab

# 2. Install ROS bridge, audio playback tools, Python deps, and native Node-RED (No Docker)
sudo apt update
sudo apt install -y ros-jazzy-rosbridge-server alsa-utils pulseaudio-utils espeak-ng nodejs npm python3-numpy

cd ~/turtlebot3_ws/src/trainerbot_rehab
pip install --break-system-packages -r requirements_laptop.txt

sudo npm install -g --unsafe-perm node-red
mkdir -p ~/.node-red && cd ~/.node-red
npm install @flowfuse/node-red-dashboard node-red-contrib-ros2-suite

# 3. Create your private .env (ignored by .gitignore)
cd ~/turtlebot3_ws/src/trainerbot_rehab
cp .env.example .env
nano .env   # Set GEMINI_API_KEY, GEMINI_LIVE_MODEL=gemini-3.8-live, ROS_DOMAIN_ID, ROBOT_IP

# 4. Build the ROS 2 package
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select trainerbot_rehab
source install/setup.bash
```

### B. TurtleBot3 Raspberry Pi Setup (SSH)

```bash
ssh ubuntu@<ROBOT_IP>

# 1. Clone the repository directly onto the Pi
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone https://github.com/SantiagoMartinezHernandez/WTrainer.git trainerbot_rehab

# 2. Install lightweight Pi hardware/audio packages (no MediaPipe/Gemini needed on Pi)
sudo apt update
sudo apt install -y alsa-utils espeak-ng python3-gpiozero rpi.gpio-common ros-jazzy-usb-cam
sudo usermod -aG audio,video,dialout $USER

# 3. Enable I2S audio in /boot/firmware/config.txt
sudo nano /boot/firmware/config.txt
# Uncomment / add:
#   dtparam=i2s=on
#   dtoverlay=googlevoicehat-soundcard

sudo reboot
```

After the Pi reboots, SSH back in and build:
```bash
ssh ubuntu@<ROBOT_IP>
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select trainerbot_rehab
source install/setup.bash
arecord -l   # Confirm INMP441 mic card/device (typically plughw:0,0)
```

---

## 4. Running the System

Ensure **TurtleBot3**, **Laptop**, and **Phone** are on the same Wi-Fi network (and Laptop firewall allows ROS 2 + port `1880`).

On the **Laptop**:
```bash
chmod +x ~/turtlebot3_ws/src/trainerbot_rehab/scripts/start_system.sh
bash ~/turtlebot3_ws/src/trainerbot_rehab/scripts/start_system.sh
```

1. The script launches Pi hardware bringup over SSH, starts Laptop Brain nodes, and starts Node-RED.
2. Once ready, the **victory fanfare chime** plays and the coach greets you.
3. Open `http://<LAPTOP_IP>:1880/dashboard/page1` on your **Phone**.

### Switching Audio to the TurtleBot3 (Once MAX98357A is Wired)
- On Pi: `ros2 launch trainerbot_rehab robot_bringup.launch.py enable_speaker:=true`
- On Laptop: `ros2 launch trainerbot_rehab laptop_brain.launch.py enable_laptop_speaker:=false`

---

## 5. Additional Documentation
- Detailed wiring diagrams & troubleshooting: [`docs/WIRING_AND_SETUP.md`](docs/WIRING_AND_SETUP.md)
- Exercise catalog: [`config/exercise_catalog.json`](config/exercise_catalog.json)
