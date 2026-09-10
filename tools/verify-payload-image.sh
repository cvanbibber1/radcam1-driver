#!/bin/bash
# Check that a payload image contains a working payload, and no identity.
#
#   sudo tools/verify-payload-image.sh IMAGE.img.gz
#   sudo tools/verify-payload-image.sh /dev/sda        # a flashed drive
#
# Two questions, and both matter. Is everything needed to be a camera present -
# including the pieces that are easy to lose, like the locally built libcamera
# that shadows the Debian package. And is everything that identifies the build
# machine gone - because a clone that kept its donor's SSH host keys is a
# security problem that will not announce itself.
set -euo pipefail

SRC="${1:-}"
[ -n "$SRC" ] || { sed -n '2,10p' "$0"; exit 2; }
[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }

MNT=/mnt/radcam-verify
LOOP=""
TMPIMG=""
PASS=0; FAIL=0

cleanup() {
    set +e
    umount -R "$MNT" 2>/dev/null
    [ -n "$LOOP" ] && losetup -d "$LOOP" 2>/dev/null
    [ -n "$TMPIMG" ] && rm -f "$TMPIMG"
}
trap cleanup EXIT

ok()   { printf "  \033[32mPASS\033[0m  %s\n" "$1"; PASS=$((PASS+1)); }
bad()  { printf "  \033[31mFAIL\033[0m  %s\n" "$1"; FAIL=$((FAIL+1)); }
have() { [ -e "$MNT$1" ] && ok "${2:-$1}" || bad "${2:-$1} missing"; }
gone() { [ -e "$MNT$1" ] && bad "${2:-$1} is present and should not be" || ok "${2:-$1} absent"; }

# -- open it ---------------------------------------------------------------
if [ -b "$SRC" ]; then
    DEV="$SRC"
    BOOTP="${DEV}1"; [ -b "$BOOTP" ] || BOOTP="${DEV}p1"
    ROOTP="${DEV}2"; [ -b "$ROOTP" ] || ROOTP="${DEV}p2"
else
    case "$SRC" in
        *.gz) TMPIMG=$(mktemp /var/tmp/radcam-verify-XXXX.img)
              echo "--- decompressing"
              gzip -dc "$SRC" > "$TMPIMG"; TARGET="$TMPIMG" ;;
        *.xz) TMPIMG=$(mktemp /var/tmp/radcam-verify-XXXX.img)
              echo "--- decompressing"
              xz -dc "$SRC" > "$TMPIMG"; TARGET="$TMPIMG" ;;
        *)    TARGET="$SRC" ;;
    esac
    LOOP=$(losetup --show -fP "$TARGET")
    BOOTP="${LOOP}p1"; ROOTP="${LOOP}p2"
fi

mkdir -p "$MNT"
mount -o ro "$ROOTP" "$MNT"
mount -o ro "$BOOTP" "$MNT/boot/firmware"

echo
echo "=== flight software ==="
have /opt/radcam/radcam/stp/experiment.py    "radcam package (STP)"
have /usr/local/bin/radcamctl                "radcamctl"
have /etc/radcam/config.json                 "payload config"
have /etc/systemd/system/radcamd.service     "radcamd unit"
have /etc/systemd/system/radcam-calibration.service "calibration unit"
have /boot/firmware/overlays/ar1335.dtbo     "AR1335 device tree overlay"
have /usr/local/share/libcamera/ipa/rpi/pisp/ar1335.json "AR1335 libcamera tuning"

# The kernel ships modules compressed, so accept either form rather than
# hard-coding the extension - a check that fails on a working image trains
# people to ignore it.
if ls "$MNT"/lib/modules/*/updates/ar1335.ko* >/dev/null 2>&1; then
    ok "AR1335 kernel module ($(basename "$(ls "$MNT"/lib/modules/*/updates/ar1335.ko* | head -1)"))"
else
    bad "AR1335 kernel module missing"
fi

if ls "$MNT"/usr/local/lib/*/libcamera.so* >/dev/null 2>&1; then
    ok "locally built libcamera (shadows the Debian package)"
else
    bad "locally built libcamera missing - the Debian one has no AR1335 CamHelper"
fi

echo
echo "=== identity removed ==="
if [ -f "$MNT/etc/machine-id" ] && [ ! -s "$MNT/etc/machine-id" ]; then
    ok "machine-id present but empty (systemd repopulates on boot)"
elif [ ! -e "$MNT/etc/machine-id" ]; then
    bad "machine-id file absent entirely - should exist and be empty"
else
    bad "machine-id is populated: $(cat "$MNT/etc/machine-id")"
fi
[ "$(ls "$MNT"/etc/ssh/ssh_host_*_key 2>/dev/null | wc -l)" -eq 0 ] \
    && ok "no SSH host keys" || bad "SSH host keys present - clones would share an identity"
[ "$(ls "$MNT"/var/lib/radcam/dosimeter-cal.json.* 2>/dev/null | wc -l)" -eq 0 ] \
    && ok "no dosimeter calibration (it is per-Pi)" \
    || bad "dosimeter calibration present - it describes the build machine's board"
gone /etc/sudoers.d/claude-autonomy "passwordless sudo dev rule"
gone /home/rad/.vscode-server       "vscode server"
gone /var/swap                      "swapfile"
[ "$(ls "$MNT"/etc/NetworkManager/system-connections/* 2>/dev/null | wc -l)" -eq 0 ] \
    && ok "no saved wifi credentials" || bad "wifi credentials present"

echo
echo "=== first-boot provisioning ==="
have /usr/local/sbin/radcam-firstboot.sh "first-boot script"
[ -L "$MNT/etc/systemd/system/sysinit.target.wants/radcam-firstboot.service" ] \
    && ok "first-boot service enabled" || bad "first-boot service not enabled"
gone /var/lib/radcam/.firstboot-done "first-boot done-stamp"
have /boot/firmware/radcam-unit.txt "radcam-unit.txt"
# An unset unit number is correct for a master image and wrong for a drive
# about to go into a payload, so which one this is decides whether it is a
# failure. Treating them the same would either bless an unstamped drive or
# cry wolf on every build.
UNIT=$(grep -v '^#' "$MNT/boot/firmware/radcam-unit.txt" 2>/dev/null \
       | tr -dc '0-9' | head -c 3)
if [ -n "$UNIT" ]; then
    ok "unit number set to $UNIT (boots as radcam$UNIT)"
elif [ -b "$SRC" ]; then
    bad "no unit number - this drive would boot as radcam-unconfigured"
else
    printf "  \033[33mNOTE\033[0m  no unit number, as expected for a master image\n"
    printf "        stamp it at flash time:  flash-payload-image.sh ... --unit N\n"
fi

echo
echo "=== boot consistency ==="
# The classic cloned-Pi failure: fstab and cmdline naming a PARTUUID the disk
# does not have, which boots to a rootwait timeout with no useful message.
CMDLINE_UUID=$(grep -o 'root=PARTUUID=[^ ]*' "$MNT/boot/firmware/cmdline.txt" | cut -d= -f3)
FSTAB_UUID=$(awk '$2=="/" {print $1}' "$MNT/etc/fstab" | cut -d= -f2)
ACTUAL=$(blkid -s PARTUUID -o value "$ROOTP" 2>/dev/null)
echo "    cmdline.txt : $CMDLINE_UUID"
echo "    fstab       : $FSTAB_UUID"
echo "    actual      : $ACTUAL"
[ "$CMDLINE_UUID" = "$ACTUAL" ] && ok "cmdline.txt matches the root partition" \
                                || bad "cmdline.txt PARTUUID does not match - will not boot"
[ "$FSTAB_UUID" = "$ACTUAL" ] && ok "fstab matches the root partition" \
                              || bad "fstab PARTUUID does not match"

echo
if [ "$FAIL" -eq 0 ]; then
    printf "\033[32m=== %d checks passed, image is deployable ===\033[0m\n" "$PASS"
    exit 0
else
    printf "\033[31m=== %d passed, %d FAILED - do not deploy ===\033[0m\n" "$PASS" "$FAIL"
    exit 1
fi
