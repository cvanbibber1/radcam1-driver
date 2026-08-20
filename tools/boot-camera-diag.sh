#!/usr/bin/env bash
# Capture the camera bus state as early in boot as possible.
#
# The outstanding question is whether the AR1335 comes up after a genuine COLD
# power cycle - one that drops the 3V3 rail so the module's power-on-reset
# re-runs with its enable already high. Every reboot so far has been warm, which
# never drops that rail.
#
# When that cold cycle happens, the interesting moments are the first few
# seconds. This records them to a persistent log so the answer is captured even
# if nobody is watching, which is the point of a zero-intervention payload.
#
# Installed as radcam-bootdiag.service (oneshot, early boot).

set -u

LOG=/var/log/radcam-bootdiag.log
exec >>"$LOG" 2>&1

echo
echo "================================================================"
echo "boot diagnostic  $(date -Is)  uptime $(cut -d' ' -f1 /proc/uptime)s"
echo "================================================================"

echo "--- enable pins (should be HIGH from boot: regulator-always-on) ---"
for p in 34 35 46 48; do pinctrl get "$p"; done

echo "--- camera I2C pins ---"
for p in 38 39 40 41; do pinctrl get "$p"; done

echo "--- camera regulators ---"
grep -E 'cam[01]_reg' /sys/kernel/debug/regulator/regulator_summary 2>/dev/null \
    || echo "  (regulator_summary unavailable)"

echo "--- I2C bus scans ---"
modprobe i2c-dev 2>/dev/null
for spec in "CAM0:6" "CAM1:4" "HDR:1"; do
    IFS=: read -r name bus <<<"$spec"
    if [ -e "/dev/i2c-$bus" ]; then
        hits=$(i2cdetect -y -a -r "$bus" 2>/dev/null | tail -n +2 \
               | grep -oE '\b[0-9a-f]{2}\b' | grep -vE '^[0-7]0$' | tr '\n' ' ')
        printf '  %-5s i2c-%-2s : %s\n' "$name" "$bus" "${hits:-nothing}"
    else
        printf '  %-5s i2c-%-2s : bus missing\n' "$name" "$bus"
    fi
done

echo "--- AR1335 model ID probe (expect 0x01 0x53) ---"
for bus in 6 4; do
    [ -e "/dev/i2c-$bus" ] || continue
    id=$(i2ctransfer -y "$bus" -f w2@0x36 0x30 0x00 r2 2>&1)
    printf '  i2c-%-2s reg 0x3000 : %s\n' "$bus" "$id"
done

echo "--- driver messages ---"
dmesg | grep -iE 'ar1335|rp1-cfe' | grep -v 'dependency cycle' | tail -12 \
    || echo "  (none yet)"

echo "--- bound devices ---"
ls /sys/bus/i2c/drivers/ar1335/ 2>/dev/null | grep -E '^[0-9]+-' \
    || echo "  (driver bound to nothing)"

echo "--- end ---"
