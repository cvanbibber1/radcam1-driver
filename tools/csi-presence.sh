#!/usr/bin/env bash
# Electrical presence test for a module on the CAM/DISP FPC connectors.
#
# Idea: the Pi's internal pull-up (~50 kOhm) holds SDA/SCL high whether or not a
# module is attached, so an idle-high reading proves nothing. A camera module
# carries its own much stronger I2C pull-ups (typically 1.8-10 kOhm to 3V3).
# So: switch the pin to a plain input with the *internal pull-down* enabled.
#   - module attached  -> external pull-up wins, pin still reads HIGH
#   - nothing attached -> internal pull-down wins, pin reads LOW
#
# The pin is switched back to its I2C alt function afterwards.

set -u

echo "=== CSI connector presence test — $(date -Is) ==="
echo

# name : pin : alt-function-to-restore
PINS=("CAM0_SDA:38:a3" "CAM0_SCL:39:a3" "CAM1_SDA:40:a2" "CAM1_SCL:41:a2")

probe() {
    local name=$1 pin=$2 alt=$3
    sudo pinctrl set "$pin" ip pd
    sleep 0.1
    local lvl
    lvl=$(pinctrl get "$pin" | grep -oE '\| (hi|lo)' | awk '{print $2}')
    sudo pinctrl set "$pin" "$alt" pu          # restore I2C function
    if [ "$lvl" = "hi" ]; then
        printf '  %-9s (line %-2s) : HIGH against internal pull-down -> external pull-up present (module attached)\n' "$name" "$pin"
    else
        printf '  %-9s (line %-2s) : LOW  -> no external pull-up (nothing attached / unpowered)\n' "$name" "$pin"
    fi
}

echo "--- with sensor ENABLE (IO1) asserted HIGH ---"
sudo pinctrl set 35 op dh; sudo pinctrl set 48 op dh
sleep 0.5
for spec in "${PINS[@]}"; do
    IFS=: read -r n p a <<<"$spec"
    probe "$n" "$p" "$a"
done

echo
echo "--- reference: 40-pin header I2C (i2c-1), which has a known live device ---"
# GPIO2/3 are the ARM I2C; a responding LTC2485 sits on this bus.
for spec in "HDR_SDA:2:a3" "HDR_SCL:3:a3"; do
    IFS=: read -r n p a <<<"$spec"
    probe "$n" "$p" "$a"
done

echo
echo "--- I2C line states restored ---"
for p in 38 39 40 41 2 3; do pinctrl get $p; done
