#!/usr/bin/env bash
# Pi Hardware Diagnostic Script for WTrainer / TrainerBot Rehab
# Run directly on the TurtleBot3 Raspberry Pi via SSH BEFORE full system launch:
#   bash ~/turtlebot3_ws/src/trainerbot_rehab/scripts/hardware_check.sh

set +u
set -eo pipefail

GPIO_PIN="${1:-12}"

echo "============================================================"
echo " WTrainer Pi Hardware Check"
echo "============================================================"

echo ""
echo "[1/5] Environment & Serial Devices (OpenCR + LiDAR)"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>} | LDS_MODEL=${LDS_MODEL:-<unset>}"
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/gpiomem* 2>/dev/null || echo "  [WARN] Some /dev/tty* or /dev/gpiomem* devices not found."
echo "  User groups: $(id -nG)"

echo ""
echo "[2/5] USB Camera (/dev/video*)"
if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl --list-devices || true
else
    ls -l /dev/video* 2>/dev/null || echo "  [WARN] No /dev/video* devices found!"
fi

echo ""
echo "[3/5] Camera Tilt Servo Test (GPIO ${GPIO_PIN})"
python3 - <<PYEOF
pin = int("${GPIO_PIN}")
try:
    from gpiozero import AngularServo, Device
    import time
    print(f"  Pin factory: {Device.pin_factory}")
    print(f"  Testing gpiozero AngularServo on GPIO {pin} (1.0-2.0 ms pulses)...")
    s = AngularServo(pin, min_angle=-45.0, max_angle=45.0, min_pulse_width=0.001, max_pulse_width=0.002)
    for angle in (0.0, 40.0, -40.0, 0.0):
        print(f"    -> Moving to {angle:+.0f} deg")
        s.angle = angle
        time.sleep(1.0)
    s.close()
    print("  [OK] Servo test commands sent via gpiozero!")
    print("  If it buzzes without moving: check 5V power supply, shared GND, and mechanical binding.")
except Exception as e:
    print(f"  [FAIL] gpiozero servo failed: {e}")
    print("  Tip: On Ubuntu 24.04 Pi, run: sudo apt install -y python3-lgpio")
PYEOF

echo ""
echo "[4/5] INMP441 I2S Microphone Check"
arecord -l || true
MIC_DEV="$(arecord -l 2>/dev/null | awk -F'[: ]+' '/^card /{print "plughw:"$2",0"; exit}')"
if [ -n "$MIC_DEV" ]; then
    echo "  Recording 2 seconds from $MIC_DEV..."
    if arecord -q -D "$MIC_DEV" -f S16_LE -r 16000 -c 1 -d 2 /tmp/wtrainer_mic_test.wav; then
        python3 - <<'PYEOF'
import wave, struct
with wave.open('/tmp/wtrainer_mic_test.wav', 'rb') as w:
    raw = w.readframes(w.getnframes())
samples = struct.unpack(f'<{len(raw)//2}h', raw)
peak = max(abs(s) for s in samples) if samples else 0
print(f"  [OK] Captured {len(samples)} samples; peak amplitude = {peak} / 32767")
if peak == 0:
    print("  [WARN] Peak is 0! Check INMP441 wiring (VDD=3.3V, L/R=GND, SCK=Pin12, WS=Pin35, SD=Pin38) and /boot/firmware/config.txt.")
PYEOF
    else
        echo "  [FAIL] arecord failed on $MIC_DEV"
    fi
else
    echo "  [WARN] No ALSA capture device found. Check /boot/firmware/config.txt (dtparam=i2s=on, dtoverlay=googlevoicehat-soundcard)."
fi

echo ""
echo "[5/5] Summary"
echo "  If steps [1]-[4] succeeded, run the Laptop startup script."
echo "============================================================"
