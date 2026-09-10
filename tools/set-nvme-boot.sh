#!/bin/bash
# Point a Pi 5's bootloader at NVMe first.
#
# This is the one part of a payload that a disk image cannot carry: BOOT_ORDER
# lives in the Pi's own SPI EEPROM, not on any drive. A perfectly flashed SSD
# in a Pi that still tries SD first will boot whatever is in the SD slot, or
# sit at the diagnostic screen - which looks exactly like a bad flash.
#
#   sudo tools/set-nvme-boot.sh [--show]
set -euo pipefail

[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }

# BOOT_ORDER digits are read right to left: 6=NVMe, 1=SD, 4=USB-MSD, f=restart.
# 0xf416 therefore means NVMe, then SD, then USB, then loop - which keeps the
# SD slot as a recovery path rather than removing it.
WANT=0xf416

echo "=== current bootloader configuration ==="
rpi-eeprom-config | sed 's/^/    /'

if [ "${1:-}" = "--show" ]; then exit 0; fi

CURRENT=$(rpi-eeprom-config | awk -F= '/^BOOT_ORDER/ {print $2}')
if [ "$CURRENT" = "$WANT" ]; then
    echo
    echo "BOOT_ORDER is already $WANT (NVMe, then SD, then USB). Nothing to do."
    exit 0
fi

echo
echo "Changing BOOT_ORDER: ${CURRENT:-unset} -> $WANT"
read -r -p "Proceed? [y/N] " ok
[ "$ok" = "y" ] || { echo "aborted"; exit 1; }

TMP=$(mktemp)
rpi-eeprom-config > "$TMP"
if grep -q '^BOOT_ORDER=' "$TMP"; then
    sed -i "s/^BOOT_ORDER=.*/BOOT_ORDER=$WANT/" "$TMP"
else
    echo "BOOT_ORDER=$WANT" >> "$TMP"
fi
rpi-eeprom-config --apply "$TMP"
rm -f "$TMP"

echo
echo "Applied. It takes effect on the next reboot."
rpi-eeprom-config | sed 's/^/    /'
