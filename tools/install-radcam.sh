#!/usr/bin/env bash
# Install the radcam payload software as a systemd service.
#
#   sudo bash tools/install-radcam.sh
#
# Idempotent: safe to re-run after editing the source.

set -euo pipefail

SRC="$(cd "$(dirname "$0")/.." && pwd)"
DEST=/opt/radcam
CONF=/etc/radcam/config.json

if [ "$(id -u)" -ne 0 ]; then
    echo "must run as root" >&2
    exit 1
fi

echo "installing radcam package -> $DEST"
install -d "$DEST/radcam"
install -m 0644 "$SRC"/radcam/*.py "$DEST/radcam/"
# The STP/DICE RS-422 protocol lives in a subpackage; a flat glob misses it
# and the daemon then fails to import at startup.
install -d "$DEST/radcam/stp"
install -m 0644 "$SRC"/radcam/stp/*.py "$DEST/radcam/stp/"

echo "installing radcamctl -> /usr/local/bin"
cat > /usr/local/bin/radcamctl <<'EOF'
#!/usr/bin/env bash
exec env PYTHONPATH=/opt/radcam /usr/bin/python3 -m radcam.cli "$@"
EOF
chmod 0755 /usr/local/bin/radcamctl

install -d /etc/radcam /var/lib/radcam

if [ ! -f "$CONF" ]; then
    echo "writing default config -> $CONF"
    cat > "$CONF" <<'EOF'
{
  "interval_s": 5.0,
  "warmup_s": 15.0,

  "flight_port": "/dev/ttyAMA0",
  "flight_baud": 115200,

  "_comment_mirror": "GPIO23/24 have no hardware UART on a Pi 5. Set this to a real device (e.g. /dev/ttyAMA10, after removing console=serial0 from cmdline.txt) or leave null to disable mirroring.",
  "mirror_port": null,
  "mirror_baud": 115200,

  "led_enabled": true,
  "led_brightness": 0.0,

  "tmr_fields": ["dose_rad", "cal_zero_v"]
}
EOF
else
    echo "keeping existing config $CONF"
fi

echo "installing systemd unit"
install -m 0644 "$SRC/systemd/radcamd.service" /etc/systemd/system/radcamd.service
# Ordered before radcamd: the tuning file must carry this camera's calibration
# before anything opens the camera, or the first captures use the wrong matrix.
install -m 0644 "$SRC/systemd/radcam-calibration.service" \
        /etc/systemd/system/radcam-calibration.service
systemctl daemon-reload

echo
echo "installed. next steps:"
echo "  systemctl enable --now radcam-calibration"
echo "  systemctl enable --now radcamd"
echo "  radcamctl status"
echo "  journalctl -u radcamd -f"
