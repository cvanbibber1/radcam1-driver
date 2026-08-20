#!/usr/bin/env bash
# End-to-end verification of the AR1335 pipeline.
#
# Run this once the sensor ACKs on I2C. It walks the stack bottom-up and stops
# at the first layer that fails, so the output points straight at the problem.

set -u

LOG_DIR="$(cd "$(dirname "$0")/.." && pwd)/logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/verify-camera.$(date +%Y%m%d-%H%M%S).log"
exec > >(tee "$LOG") 2>&1

pass() { printf '  \033[32mPASS\033[0m  %s\n' "$1"; }
fail() { printf '  \033[31mFAIL\033[0m  %s\n' "$1"; }
info() { printf '        %s\n' "$1"; }

echo "=== AR1335 pipeline verification — $(date -Is) ==="
echo

echo "[1] Sensor responds on I2C"
found=""
for spec in "CAM0:6" "CAM1:4"; do
    IFS=: read -r name bus <<<"$spec"
    id=$(sudo i2ctransfer -y "$bus" -f w2@0x36 0x30 0x00 r2 2>/dev/null)
    if [ "$id" = "0x01 0x53" ]; then
        pass "$name (i2c-$bus): model ID 0x0153"
        found="$name"
    else
        info "$name (i2c-$bus): ${id:-no response}"
    fi
done
[ -n "$found" ] || { fail "sensor did not answer on either port"; exit 1; }
echo

echo "[2] Kernel driver bound"
if dmesg | grep -q 'ar1335.*AR1335 found'; then
    pass "driver probed"
    dmesg | grep -E 'ar1335' | tail -5 | sed 's/^/        /'
else
    fail "driver did not probe"
    dmesg | grep -E 'ar1335' | tail -10 | sed 's/^/        /'
    exit 1
fi
echo

echo "[3] Media graph"
for m in /dev/media*; do
    if media-ctl -d "$m" -p 2>/dev/null | grep -q ar1335; then
        pass "$m carries the ar1335 subdev"
        media-ctl -d "$m" -p 2>/dev/null | grep -E 'entity|ar1335|rp1-cfe' | head -20 | sed 's/^/        /'
    fi
done
echo

echo "[4] libcamera enumeration"
if rpicam-hello --list-cameras 2>&1 | grep -q ar1335; then
    pass "libcamera sees the sensor"
    rpicam-hello --list-cameras 2>&1 | sed 's/^/        /'
else
    fail "libcamera does not list the camera"
    info "a tuning file is needed at /usr/share/libcamera/ipa/rpi/pisp/ar1335.json"
    rpicam-hello --list-cameras 2>&1 | sed 's/^/        /'
fi
echo

echo "[5] Frame capture per mode"
for mode in "4096:3072" "1920:1080"; do
    IFS=: read -r w h <<<"$mode"
    out="$LOG_DIR/test-${w}x${h}.jpg"
    if rpicam-still --width "$w" --height "$h" -n -t 2000 -o "$out" >/dev/null 2>&1 \
       && [ -s "$out" ]; then
        pass "${w}x${h} still -> $out ($(stat -c%s "$out") bytes)"
    else
        fail "${w}x${h} capture failed"
    fi
done
echo

echo "Log written to $LOG"
