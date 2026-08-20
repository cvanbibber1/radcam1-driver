#!/usr/bin/env bash
# Watch for the AR1335 appearing and bind the driver to it automatically.
#
# The kernel probes an I2C device once at boot. If the sensor is not answering
# at that moment - slow rails, a module that needs a cold power cycle, an FPC
# reseated while running - the driver gives up and nothing retries. For a
# zero-intervention payload that is unacceptable: the camera must come into
# service by itself whenever it becomes visible.
#
# This polls both camera buses for the AR1335 model ID and, on finding it,
# re-binds the driver so the sensor is picked up without a reboot.
#
# Run from radcam-autodetect.service (a systemd timer), or by hand:
#   sudo bash tools/ar1335-autodetect.sh --once
#   sudo bash tools/ar1335-autodetect.sh --watch 10

set -u

MODEL_ID_HI=0x01
MODEL_ID_LO=0x53
ADDR=0x36
DRIVER=/sys/bus/i2c/drivers/ar1335

# bus:enable-line, CAM0 first since that is where the module lives.
PORTS=("6:35:10" "4:48:11")

log() { printf '%s ar1335-autodetect: %s\n' "$(date -Is)" "$*"; }

sensor_present() {   # sensor_present <bus>
    local bus=$1 id
    id=$(i2ctransfer -y "$bus" -f w2@$ADDR 0x30 0x00 r2 2>/dev/null) || return 1
    [ "$id" = "$MODEL_ID_HI $MODEL_ID_LO" ]
}

already_bound() {    # already_bound <adapter-nr>
    [ -e "$DRIVER/$1-0036" ]
}

try_port() {         # try_port <bus> <enable-line> <adapter-nr>
    local bus=$1 en=$2 adapter=$3

    already_bound "$adapter" && return 0

    # Assert the enable in case nothing else is holding it; the regulator only
    # drives it while a driver is bound, which is precisely what we lack.
    pinctrl set "$en" op dh 2>/dev/null
    sleep 0.2

    sensor_present "$bus" || return 1

    log "AR1335 detected on i2c-$bus, binding driver"
    if [ ! -d "$DRIVER" ]; then
        modprobe ar1335 || { log "modprobe failed"; return 1; }
        sleep 1
        already_bound "$adapter" && { log "bound via modprobe"; return 0; }
    fi

    # Bind only. Do NOT unbind first: tearing down a sensor subdev that the
    # CSI front end has already bound makes rp1-cfe re-run its async-complete
    # path and register its video nodes a second time, which oopses in
    # media_device_register_entity. We only get here when nothing is bound,
    # so there is nothing to unbind anyway.
    if echo "$adapter-0036" > "$DRIVER/bind" 2>/dev/null; then
        log "driver bound to $adapter-0036"
        return 0
    fi

    log "bind failed for $adapter-0036"
    return 1
}

scan_once() {
    local found=1
    for spec in "${PORTS[@]}"; do
        IFS=: read -r bus en adapter <<<"$spec"
        [ -e "/dev/i2c-$bus" ] || continue
        if try_port "$bus" "$en" "$adapter"; then
            found=0
        fi
    done
    return $found
}

case "${1:---once}" in
    --once)
        scan_once && exit 0
        exit 1
        ;;
    --watch)
        interval=${2:-10}
        log "watching for AR1335 every ${interval}s"
        while true; do
            if scan_once; then
                log "sensor in service; exiting watch"
                exit 0
            fi
            sleep "$interval"
        done
        ;;
    *)
        echo "usage: $0 [--once | --watch <seconds>]" >&2
        exit 2
        ;;
esac
