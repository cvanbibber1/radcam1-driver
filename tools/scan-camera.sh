#!/usr/bin/env bash
# Bring-up probe: assert the sensor ENABLE pin on both CAM ports and scan I2C.
#
# The board wires the sensor enable to connector GPIO1 (IO1), which is
# RP1 line 35 on CAM0 and line 48 on CAM1. Stock Pi wiring uses IO0, so nothing
# drives IO1 unless we do it here.
#
# Uses the `pinctrl` tool rather than gpioset: pinctrl writes the RP1 registers
# directly and therefore works even when a regulator driver owns the line.

set -u

LOG_DIR="$(cd "$(dirname "$0")/.." && pwd)/logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/scan-camera.$(date +%Y%m%d-%H%M%S).log"

exec > >(tee "$LOG") 2>&1

echo "=== AR1335 bring-up scan — $(date -Is) ==="
echo

# CAM0 -> i2c-6, enable on line 35 | CAM1 -> i2c-4, enable on line 48
PORTS=("CAM0:6:35" "CAM1:4:48")

echo "--- GPIO state before ---"
for spec in "${PORTS[@]}"; do
    IFS=: read -r name bus en <<<"$spec"
    printf '%-5s enable line %-3s : %s\n' "$name" "$en" "$(pinctrl get "$en")"
done
echo

echo "--- Asserting enable pins HIGH ---"
for spec in "${PORTS[@]}"; do
    IFS=: read -r name bus en <<<"$spec"
    sudo pinctrl set "$en" op dh
    printf '%-5s line %-3s -> %s\n' "$name" "$en" "$(pinctrl get "$en")"
done

# Sensors need a moment after power/enable before they ACK on I2C.
sleep 0.5
echo

echo "--- Available I2C buses ---"
i2cdetect -l
echo

for spec in "${PORTS[@]}"; do
    IFS=: read -r name bus en <<<"$spec"
    echo "--- $name : i2c-$bus scan ---"
    if [ ! -e "/dev/i2c-$bus" ]; then
        echo "  /dev/i2c-$bus MISSING — is dtparam=i2c_csi_dsi* enabled in config.txt?"
        echo
        continue
    fi
    sudo i2cdetect -y "$bus"
    echo

    # Probe every address that ACKed for an AR1335 model ID (reg 0x3000 == 0x0153).
    found=$(sudo i2cdetect -y -r "$bus" 2>/dev/null \
            | tail -n +2 | cut -d: -f2 | tr ' ' '\n' | grep -E '^[0-9a-f]{2}$')
    if [ -z "$found" ]; then
        echo "  no devices ACKed on i2c-$bus"
        echo
        continue
    fi

    for addr in $found; do
        echo "  device at 0x$addr — reading model ID reg 0x3000:"
        # 16-bit register address, 16-bit value, big endian.
        id=$(sudo i2ctransfer -y "$bus" -f \
                w2@0x"$addr" 0x30 0x00 r2 2>/dev/null)
        if [ -n "$id" ]; then
            echo "    reg 0x3000 = $id   (AR1335 expects 0x01 0x53)"
        else
            echo "    read failed"
        fi
    done
    echo
done

echo "--- Camera enumeration ---"
rpicam-hello --list-cameras 2>&1 | head -40 || true
echo
echo "Log written to $LOG"
