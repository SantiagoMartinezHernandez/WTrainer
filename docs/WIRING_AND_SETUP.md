# TrainerBot Rehab — Hardware Wiring, Installation & Execution Guide

This package splits your TurtleBot3 Burger into a **safety-first 3-device architecture**:

- **TurtleBot3 Pi (1 GB RAM — Hardware + Safety + PID Motion):**
  Runs `robot_bringup.launch.py`: OpenCR wheels, local LiDAR `/scan`, USB camera, smooth 50 Hz `servo_node`, driverless ALSA `speaker_node`, INMP441 `mic_node`, and **`robot_pid_controller_node`**.
  Because the PID loop, LiDAR 0.35 m safety clamp, and a **0.6 s connection-loss watchdog** live directly on the Pi, the robot reacts at local bus speed and halts immediately if Wi-Fi drops.
- **Laptop (WSL2 / Ubuntu 24.04 — AI & Vision "Brain"):**
  Runs `laptop_brain.launch.py` (`vision_pose_node` for MediaPipe Pose, `rehab_coach_node`, `gemini_live_node` using `gemini-3.8-live`, and `rosbridge_server`) plus **native Node-RED** (no Docker).
- **Phone (User Interface):**
  Opens `http://<LAPTOP_IP>:1880/dashboard/page1` in any mobile browser.

> **Privacy:** Only anonymized joint angles and exercise names are sent to Gemini — **zero video frames or personal identifiers**.

---

## 1. Raspberry Pi Wiring Guide (INMP441 Mic + APKLVSR 3W 4Ω Speaker + Servo)

### Important Note on Your Speaker (APKLVSR 3W 4Ω Passive 2-Wire Mini Speaker)
Your APKLVSR speaker is a **passive 2-wire 4Ω, 3W dynamic speaker** (no driver software needed!).

> **Electrical Safety Rule:**
> - **Never wire a 4Ω passive speaker directly to a Raspberry Pi digital GPIO pin.** A 4Ω coil draws up to $3.3\text{V} / 4\Omega = 825\text{ mA}$, while a Pi GPIO pin is rated for **16 mA max** and can be permanently damaged.
> - **Driverless Solution (Zero OS Drivers):**
>   - **Recommended (Full 3W Room Volume):** Connect the 2-wire speaker through a tiny **PAM8403 3W 4Ω mini amplifier board** (powered from Pi 5V Pin 2/4 and GND Pin 6) fed from the Pi's **3.5 mm audio jack** (or a plug-and-play USB sound card). Zero Linux audio drivers are needed — ALSA `"default"` drives it out of the box.
>   - **Quiet Bench Test Only (No Amp Board):** You can wire a 3.5 mm TRS plug directly (`Tip` → `+`, `Sleeve` → `-`) into the Pi's 3.5 mm headphone jack; it is electrically safe, though much quieter than 3W.
> - **Servo Pin Rule:** Do **not** use GPIO 18 for a PWM servo because the INMP441 I2S microphone uses GPIO 18 (`BCLK`). Put your PWM servo signal wire on **GPIO 12 (Physical Pin 32)**.

### Wiring Diagram

```text
                       Raspberry Pi 4 / 3B+ 40-Pin Header
                      +-----------------------------------+
   3.3V (INMP441 VDD) |  1 (3.3V)            (5V)      2  | -> PAM8403 Amp 5V / Servo +5V
                      |  3 (GPIO 2)          (5V)      4  |
                      |  5 (GPIO 3)          (GND)     6  | -> Common GND (Mic + Amp + Servo)
                      |  7 (GPIO 4)          (TX)      8  |
                      |  9 (GND)             (RX)     10  |
                      | 11 (GPIO 17)         (GPIO18) 12  | -> INMP441 SCK (I2S BCLK)
                      | ...                           ... |
                      | 31 (GPIO 6)          (GPIO12) 32  | -> Servo Signal (PWM)
                      | 33 (GPIO 13)         (GND)    34  |
                      | 35 (GPIO 19)         (GPIO16) 36  | -> Pin 35 = INMP441 WS (I2S LRCLK)
                      | 37 (GPIO 26)         (GPIO20) 38  | -> Pin 38 = INMP441 SD (I2S Data)
                      | 39 (GND)             (GPIO21) 40  |
                      +-----------------------------------+

   Pi 3.5 mm Audio Jack  --->  [PAM8403 3W 4Ω Mini Amp]  --->  APKLVSR 2-Wire Speaker (+ / -)
    (Driverless ALSA)          (Powered from Pin 2 & 6)
```

### Pin-by-Pin Table

| Component | Module Pin | Pi Connection | BCM / Port | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **INMP441 Mic** | `VDD` | Pin 1 | `3.3V` | **3.3V only** (never 5V) |
| **INMP441 Mic** | `GND` | Pin 6 or 9 | `GND` | Common ground |
| **INMP441 Mic** | `L/R` | Pin 6 or 9 | `GND` | Selects Left channel |
| **INMP441 Mic** | `SCK` | Pin 12 | `GPIO 18` | I2S Bit Clock |
| **INMP441 Mic** | `WS` | Pin 35 | `GPIO 19` | I2S Frame Clock |
| **INMP441 Mic** | `SD` | Pin 38 | `GPIO 20` | I2S Data In |
| **APKLVSR 2-Wire Speaker** | `+` / `-` | PAM8403 `L+`/`L-` (or 3.5 mm Tip/Sleeve) | **3.5 mm Audio Jack** | PAM8403 `5V` → Pin 2, `GND` → Pin 6 |
| **Camera PWM Servo** *(if not Dynamixel)* | Signal | Pin 32 | `GPIO 12` | `servo_type:=pwm` (`VCC` → Pin 4, `GND` → Pin 14) |

### One-Time Pi Audio & I2S Setup (Over SSH)
```bash
sudo nano /boot/firmware/config.txt
# Uncomment / add these lines:
#   dtparam=i2s=on
#   dtparam=audio=on
#   dtoverlay=googlevoicehat-soundcard
sudo apt update && sudo apt install -y alsa-utils espeak-ng python3-gpiozero
sudo usermod -aG audio,gpio,video,dialout $USER
sudo reboot
```
After reboot, force output to the 3.5 mm jack (driverless) and verify mic + speaker:
```bash
amixer cset numid=3 1          # Route ALSA playback to 3.5 mm jack (use 0 for auto)
arecord -l                     # Note INMP441 card/device (usually plughw:0,0 or plughw:1,0)
aplay -l                       # Note 3.5 mm Headphones/bcm2835 card
espeak-ng "Speaker test ready"
```

---

## 2. Full Build & Install Instructions

### Step A — Laptop (Ubuntu 24.04 / WSL2)
1. Clone the repository into your ROS 2 workspace as `trainerbot_rehab`:
   ```bash
   mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
   git clone https://github.com/SantiagoMartinezHernandez/WTrainer.git trainerbot_rehab
   ```
2. Install Python dependencies, `rosbridge_server`, and **native Node-RED** (no Docker):
   ```bash
   cd ~/turtlebot3_ws/src/trainerbot_rehab
   pip install --break-system-packages -r requirements_laptop.txt
   sudo apt update && sudo apt install -y ros-jazzy-rosbridge-server alsa-utils pulseaudio-utils espeak-ng nodejs npm python3-numpy
   sudo npm install -g --unsafe-perm node-red
   mkdir -p ~/.node-red && cd ~/.node-red
   npm install @flowfuse/node-red-dashboard node-red-contrib-ros2
   ```
3. Configure your secret `.env` safely (never committed to Git):
   ```bash
   cd ~/turtlebot3_ws/src/trainerbot_rehab
   cp .env.example .env
   nano .env   # Set GEMINI_API_KEY=..., GEMINI_LIVE_MODEL=gemini-3.8-live, ROS_DOMAIN_ID=108, ROBOT_IP=...
   ```
4. Build on the Laptop:
   ```bash
   cd ~/turtlebot3_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --packages-select trainerbot_rehab
   source install/setup.bash
   ```

### Step B — TurtleBot3 Raspberry Pi
1. SSH into the Pi and clone directly from GitHub:
   ```bash
   ssh ubuntu@<ROBOT_IP>
   mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
   git clone https://github.com/SantiagoMartinezHernandez/WTrainer.git trainerbot_rehab
   ```
2. Install lightweight Pi dependencies and build (no MediaPipe or Gemini SDK needed on the Pi!):
   ```bash
   sudo apt update && sudo apt install -y alsa-utils espeak-ng python3-gpiozero rpi.gpio-common ros-jazzy-usb-cam
   cd ~/turtlebot3_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --packages-select trainerbot_rehab
   source install/setup.bash
   ```

---

## 3. Execution Steps

1. Connect **TurtleBot3**, **Laptop**, and **Phone** to the same Wi-Fi (`ProFab`) with Laptop/WSL firewalls disabled.
2. Run the master startup script on the **Laptop**:
   ```bash
   chmod +x ~/turtlebot3_ws/src/trainerbot_rehab/scripts/start_system.sh
   bash ~/turtlebot3_ws/src/trainerbot_rehab/scripts/start_system.sh
   ```
   *(Or pass custom hardware flags on the Pi manually via `ros2 launch trainerbot_rehab robot_bringup.launch.py alsa_mic:=plughw:1,0 alsa_speaker:=default servo_type:=pwm`.)*
3. Wait for the robot to play the **victory fanfare chime** and speak its welcome prompt.
4. On your **Phone**, open the URL printed by the script:
   `http://<LAPTOP_IP>:1880/dashboard/page1`
5. Tap **Start Auto-Framing PID** to center the patient (the Pi handles PID motion, LiDAR 1 m spacing, and connection-loss failsafe locally), choose an exercise or tap **Suggest Me a Routine (AI)**, and say **"Stop"** anytime to halt immediately.

---

## 4. Temporary Laptop Audio vs. Final MAX98357A Setup (In 2 Days)

- **Right now (before MAX98357A arrives):**
  `laptop_brain.launch.py` launches `speaker_node` on the **Laptop** (`enable_laptop_speaker:=true`) and `robot_bringup.launch.py` leaves the Pi speaker disabled (`enable_speaker:=false`). Every chime, local TTS cue, and `gemini-3.8-live` voice reply plays directly out of your Laptop speakers.
  *(Make sure `alsa-utils` and `espeak-ng` are installed on the Laptop/WSL: `sudo apt install -y alsa-utils espeak-ng`.)*

- **When your MAX98357A arrives:**
  1. Wire the MAX98357A to the Pi:
     - `VIN` → **Pin 2 (`5V`)** | `GND` → **Pin 6 (`GND`)**
     - `BCLK` → **Pin 12 (`GPIO 18`)** *(shared with INMP441 `SCK`)*
     - `LRC` → **Pin 35 (`GPIO 19`)** *(shared with INMP441 `WS`)*
     - `DIN` → **Pin 40 (`GPIO 21`)**
     - Screw your APKLVSR 2-wire speaker into **`+` (Red)** and **`-` (Black)** on the MAX98357A.
  2. Flip the two flags to move audio back to the TurtleBot3:
     ```bash
     # On Pi:
     ros2 launch trainerbot_rehab robot_bringup.launch.py enable_speaker:=true alsa_speaker:=default
     # On Laptop:
     ros2 launch trainerbot_rehab laptop_brain.launch.py enable_laptop_speaker:=false
     ```
