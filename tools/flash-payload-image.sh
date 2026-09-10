#!/bin/bash
# Write a payload image to an SSD and stamp it with its unit number.
#
#   sudo tools/flash-payload-image.sh IMAGE /dev/sdX --unit 2 [--verify]
#
# Refuses to write to a mounted disk, to the disk it is running from, or to
# anything that does not look like removable/target media unless --force is
# given. Five drives in a row is exactly the situation where a habit of typing
# the device name quickly meets a machine whose own root disk is one letter
# away.
set -euo pipefail

UNIT=""
VERIFY=0
FORCE=0
IMAGE=""
TARGET=""

while [ $# -gt 0 ]; do
    case "$1" in
        --unit) UNIT="$2"; shift 2 ;;
        --verify) VERIFY=1; shift ;;
        --force) FORCE=1; shift ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) if [ -z "$IMAGE" ]; then IMAGE="$1"; elif [ -z "$TARGET" ]; then TARGET="$1"; else
               echo "unexpected argument: $1" >&2; exit 2; fi; shift ;;
    esac
done

[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }
[ -n "$IMAGE" ] && [ -n "$TARGET" ] || { sed -n '2,12p' "$0"; exit 2; }
[ -f "$IMAGE" ] || { echo "no such image: $IMAGE" >&2; exit 1; }
[ -b "$TARGET" ] || { echo "not a block device: $TARGET" >&2; exit 1; }

# -- safety ----------------------------------------------------------------
ROOT_SRC=$(findmnt -no SOURCE / | sed 's/p\?[0-9]*$//')
if [ "$TARGET" = "$ROOT_SRC" ]; then
    echo "REFUSING: $TARGET is the disk this system is running from." >&2
    exit 1
fi
if lsblk -no MOUNTPOINT "$TARGET" | grep -q .; then
    echo "REFUSING: $TARGET has mounted partitions:" >&2
    lsblk -o NAME,SIZE,MOUNTPOINT "$TARGET" >&2
    echo "Unmount them first, or pass --force if you are certain." >&2
    [ "$FORCE" -eq 1 ] || exit 1
fi

SIZE=$(lsblk -bdno SIZE "$TARGET")
MODEL=$(lsblk -dno MODEL "$TARGET" | xargs || true)
echo "=== flash target ==="
echo "    device : $TARGET"
echo "    model  : ${MODEL:-unknown}"
echo "    size   : $(numfmt --to=iec "$SIZE")"
echo "    image  : $(basename "$IMAGE")"
echo "    unit   : ${UNIT:-<unchanged, image default>}"
echo
lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINT "$TARGET" | sed 's/^/    /'
echo
read -r -p "This ERASES $TARGET completely. Type the device name to confirm: " CONFIRM
[ "$CONFIRM" = "$TARGET" ] || { echo "aborted"; exit 1; }

# -- checksum --------------------------------------------------------------
if [ -f "$IMAGE.sha256" ]; then
    echo "--- verifying image checksum"
    (cd "$(dirname "$IMAGE")" && sha256sum -c "$(basename "$IMAGE").sha256") \
        || { echo "image checksum FAILED; not flashing" >&2; exit 1; }
fi

# -- write -----------------------------------------------------------------
echo "--- writing"
case "$IMAGE" in
    *.gz) gzip -dc "$IMAGE" | dd of="$TARGET" bs=4M conv=fsync status=progress ;;
    *.xz) xz -dc "$IMAGE"   | dd of="$TARGET" bs=4M conv=fsync status=progress ;;
    *)    dd if="$IMAGE" of="$TARGET" bs=4M conv=fsync status=progress ;;
esac
sync
partprobe "$TARGET" 2>/dev/null || true
sleep 2

# -- stamp the unit number -------------------------------------------------
if [ -n "$UNIT" ]; then
    echo "--- setting unit number to $UNIT"
    BOOTPART="${TARGET}1"
    [ -b "$BOOTPART" ] || BOOTPART="${TARGET}p1"
    TMPMNT=$(mktemp -d)
    mount "$BOOTPART" "$TMPMNT"
    echo "$UNIT" > "$TMPMNT/radcam-unit.txt"
    sync
    umount "$TMPMNT"
    rmdir "$TMPMNT"
    echo "    this SSD will come up as radcam${UNIT}"
fi

# -- verify ----------------------------------------------------------------
if [ "$VERIFY" -eq 1 ]; then
    echo "--- verifying written data"
    IMG_BYTES=$(case "$IMAGE" in
        *.gz) gzip -l "$IMAGE" | awk 'NR==2 {print $2}' ;;
        *) stat -c %s "$IMAGE" ;;
    esac)
    # Compare only the bytes the image actually wrote; everything past that is
    # whatever the drive had before and is about to be claimed by the first
    # boot's partition expansion.
    case "$IMAGE" in
        *.gz) SUM_A=$(gzip -dc "$IMAGE" | sha256sum | cut -d' ' -f1) ;;
        *) SUM_A=$(sha256sum "$IMAGE" | cut -d' ' -f1) ;;
    esac
    SUM_B=$(head -c "$IMG_BYTES" "$TARGET" | sha256sum | cut -d' ' -f1)
    if [ "$SUM_A" = "$SUM_B" ]; then
        echo "    verified: $IMG_BYTES bytes match"
    else
        echo "    VERIFY FAILED - do not deploy this drive" >&2
        exit 1
    fi
fi

echo
echo "=== done ==="
echo "Next, on the Pi this SSD goes into:"
echo "  1. set the bootloader to try NVMe first:  sudo tools/set-nvme-boot.sh"
echo "  2. boot it, then:  sudo radcamctl calibrate   (dosimeter is per-Pi)"
