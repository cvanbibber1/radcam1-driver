#!/bin/bash
# First boot of a freshly flashed payload.
#
# A disk image is a clone, and three things in a clone are wrong by definition:
# the machine's identity, the size of the root filesystem, and which unit it is.
# This fixes all three, then removes itself. It is deliberately conservative -
# every step is skipped rather than retried if it looks already done, because
# this runs before anything else on a payload nobody is watching.
#
# Anything that fails here is logged and does not stop the boot. A payload that
# comes up with a stale hostname is a nuisance; one that will not come up at all
# is a dead camera.
set -uo pipefail

LOG=/var/log/radcam-firstboot.log
exec > >(tee -a "$LOG") 2>&1
echo "=== radcam first boot: $(date -Is) ==="

BOOT=/boot/firmware
UNIT_FILE="$BOOT/radcam-unit.txt"
STAMP=/var/lib/radcam/.firstboot-done

step() { echo "--- $*"; }
warn() { echo "!!! $*"; }

# -- 1. identity ------------------------------------------------------------
# Every clone carries the donor's machine-id and SSH host keys. Left alone,
# five payloads are indistinguishable to anything that keys on them - DHCP
# leases, journald, and any host that has ever trusted one of them by SSH.

step "machine-id"
if [ ! -s /etc/machine-id ]; then
    systemd-machine-id-setup && echo "    generated $(cat /etc/machine-id)"
else
    echo "    already set: $(cat /etc/machine-id)"
fi

step "SSH host keys"
if ! ls /etc/ssh/ssh_host_*_key >/dev/null 2>&1; then
    ssh-keygen -A && echo "    regenerated"
else
    echo "    already present"
fi

# -- 2. which unit is this --------------------------------------------------
# The unit number lives on the FAT boot partition so it can be set from any
# machine that can read an SD card or an SSD - Windows included - without
# needing to mount ext4 or boot the payload first.

step "hostname"
if [ -f "$UNIT_FILE" ]; then
    UNIT=$(tr -dc '0-9' < "$UNIT_FILE" | head -c 3)
fi
if [ -n "${UNIT:-}" ]; then
    NEW="radcam${UNIT}"
    CURRENT=$(hostname)
    if [ "$NEW" != "$CURRENT" ]; then
        echo "$NEW" > /etc/hostname
        # Keep /etc/hosts consistent or sudo warns and name resolution for the
        # local host breaks, which is confusing out of all proportion.
        sed -i "s/\b${CURRENT}\b/${NEW}/g" /etc/hosts
        hostnamectl set-hostname "$NEW" 2>/dev/null || true
        echo "    $CURRENT -> $NEW"
    else
        echo "    already $NEW"
    fi
else
    warn "NO UNIT NUMBER SET in $UNIT_FILE"
    warn "  this payload is running as '$(hostname)'"
    warn "  put a digit in that file on the boot partition and reboot, or run"
    warn "  hostnamectl set-hostname radcamN"
fi

# -- 3. fill the disk -------------------------------------------------------
# The image is built just big enough to hold the system, so it writes quickly
# to any SSD. The root partition then has to grow to whatever it landed on.

step "expand root filesystem"
ROOT_DEV=$(findmnt -no SOURCE /)                 # e.g. /dev/nvme0n1p2
case "$ROOT_DEV" in
    /dev/nvme*p*) DISK=${ROOT_DEV%p*}; PARTNUM=${ROOT_DEV##*p} ;;
    /dev/mmcblk*p*) DISK=${ROOT_DEV%p*}; PARTNUM=${ROOT_DEV##*p} ;;
    /dev/sd*)     DISK=$(echo "$ROOT_DEV" | sed 's/[0-9]*$//'); PARTNUM=$(echo "$ROOT_DEV" | grep -o '[0-9]*$') ;;
    *) DISK=""; PARTNUM="" ;;
esac

if [ -n "$DISK" ] && [ -n "$PARTNUM" ]; then
    # Only grow if there is something worth growing into. Comparing the
    # partition end against the disk end avoids running parted on a filesystem
    # that is already full-size, which is the normal case on every boot after
    # this one.
    DISK_SECTORS=$(blockdev --getsz "$DISK")
    PART_END=$(partx -g -o END "$ROOT_DEV" 2>/dev/null | tr -d ' ')
    SLACK=$(( DISK_SECTORS - PART_END ))
    if [ "${SLACK:-0}" -gt 2097152 ]; then        # more than ~1 GiB spare
        echo "    growing $ROOT_DEV into $((SLACK / 2097152)) GiB of free space"
        parted -s "$DISK" resizepart "$PARTNUM" 100% && partprobe "$DISK"
        resize2fs "$ROOT_DEV"
    else
        echo "    already fills the disk"
    fi
else
    warn "could not identify the root disk from $ROOT_DEV; not expanding"
fi

# -- 4. swap ----------------------------------------------------------------
# The swapfile is excluded from the image because it is 2 GB of nothing.
step "swap"
if [ ! -f /var/swap ] && command -v dphys-swapfile >/dev/null; then
    dphys-swapfile setup && dphys-swapfile swapon && echo "    recreated"
else
    echo "    nothing to do"
fi

# -- 5. done ----------------------------------------------------------------
mkdir -p /var/lib/radcam
date -Is > "$STAMP"
systemctl disable radcam-firstboot.service 2>/dev/null || true

echo
echo "=== first boot complete: $(hostname) ==="
echo "REMINDER: the dosimeter is calibrated per Pi, not per camera module."
echo "  This unit has no calibration yet. Run:  sudo radcamctl calibrate"
echo
