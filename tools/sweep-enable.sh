#!/usr/bin/env bash
# Sweep every combination of the two connector GPIOs (IO0, IO1) on both CAM
# ports and scan I2C after each, to find what actually wakes the AR1335.
#
# CAM0: IO0 = RP1 line 34, IO1 = line 35, I2C bus 6
# CAM1: IO0 = RP1 line 46, IO1 = line 48, I2C bus 4

set -u

LOG_DIR="$(cd "$(dirname "$0")/.." && pwd)/logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/sweep-enable.$(date +%Y%m%d-%H%M%S).log"
exec > >(tee "$LOG") 2>&1

echo "=== AR1335 enable-pin sweep — $(date -Is) ==="

scan() {          # scan <bus> -> prints ACKing addresses, empty if none
    sudo i2cdetect -y -r "$1" 2>/dev/null \
        | tail -n +2 | cut -d: -f2 | tr ' ' '\n' | grep -E '^[0-9a-f]{2}$'
}

sweep_port() {    # sweep_port <name> <bus> <io0> <io1>
    local name=$1 bus=$2 io0=$3 io1=$4
    echo
    echo "############ $name  (i2c-$bus, IO0=line $io0, IO1=line $io1) ############"
    for v0 in dl dh; do
        for v1 in dl dh; do
            sudo pinctrl set "$io0" op "$v0"
            sudo pinctrl set "$io1" op "$v1"
            sleep 1                       # let the module power up and release reset
            local hits
            hits=$(scan "$bus" | tr '\n' ' ')
            printf '  IO0=%s IO1=%s -> %s\n' \
                   "${v0/dl/LO}" "${v1/dl/LO}" \
                   "${hits:-no ACK}" | sed 's/dh/HI/g'
            if [ -n "$hits" ]; then
                for addr in $hits; do
                    local id
                    id=$(sudo i2ctransfer -y "$bus" -f w2@0x"$addr" 0x30 0x00 r2 2>/dev/null)
                    printf '      0x%s reg0x3000 = %s  (AR1335 = 0x01 0x53)\n' "$addr" "${id:-read failed}"
                done
            fi
        done
    done
    # Leave the port in the board's intended state: IO1 high = enabled.
    sudo pinctrl set "$io0" op dl
    sudo pinctrl set "$io1" op dh
}

sweep_port CAM0 6 34 35
sweep_port CAM1 4 46 48

echo
echo "Log written to $LOG"
