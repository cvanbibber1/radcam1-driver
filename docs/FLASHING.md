# Flashing a payload — five cameras from one image

This turns a bare Pi 5 + NVMe SSD into a working camera payload. The image
carries the kernel module, the device tree overlay, the locally built libcamera
with the AR1335 CamHelper, the radcam package, the STP/DICE RS-422 link and
every tool in `tools/`.

> **Why an image and not an install script.** The libcamera build in
> `/usr/local` is not reproducible from a package list — it shadows the Debian
> package and took real effort to get right. An image captures the working
> system as it is, rather than a recipe that has to work five more times.

---

## 0. What travels, and what does not

This is the part worth understanding before flashing anything, because two of
these are easy to get backwards.

| | Travels with | Why |
|---|---|---|
| **Camera colour calibration** | the **camera module** | Stored on the module's own 24C64 EEPROM. Cameras and Pis are interchangeable, so calibration follows the optics. Swap a module between units and it carries its AWB, CCM, shading and distortion with it. |
| **Dosimeter calibration** | the **Pi** | The LTC2485 is on the Pi board. A cloned calibration would be a confidently wrong number on a different board — worse than none, because an uncalibrated payload says so. **Excluded from the image; every unit needs its own.** |
| **Hostname / machine-id / SSH keys** | nothing | Regenerated on first boot. Five clones sharing an identity confuses anything that keys on it — DHCP, journald, and any host that has trusted one by SSH. |
| **BOOT_ORDER** | the **Pi's SPI EEPROM** | Not on any drive. See [§3](#3-point-the-pi-at-nvme) — this is the step that makes a good flash look like a bad one. |
| **Wifi credentials** | nothing | Deleted from the clone. A payload that joins your bench network because it was cloned from a machine that did is a surprise, not a feature. |

The development conveniences on the build machine are stripped too: the
`claude-autonomy` units, `~/.vscode-server`, and **`/etc/sudoers.d/claude-autonomy`**.
Passwordless sudo exists so an agent can work unattended on one bench Pi; it
should not be the security posture of five deployed payloads.

---

## 1. Build the image (once)

```bash
sudo tools/build-payload-image.sh
```

Roughly 4 GB of payload → a ~2 GB `.img.gz` in `/home/rad/images`, with a
`.sha256` beside it. Takes several minutes, most of it compression.

The image is sized to its contents plus 2 GB, not to the 465 GB disk it was
built from, so it writes to any SSD of 16 GB or more and expands to fill
whatever it lands on at first boot.

Useful flags: `--out DIR`, `--margin GB`, `--no-compress` (faster to build,
much slower to write five times).

---

## 2. Write it to each SSD

### On this Pi, with a USB–NVMe adapter

```bash
lsblk                                    # find the adapter, e.g. /dev/sda
sudo tools/flash-payload-image.sh /home/rad/images/radcam-payload-*.img.gz \
     /dev/sda --unit 2 --verify
```

`--unit N` stamps the unit number so the SSD comes up as `radcam<N>`.
`--verify` reads the drive back and compares SHA-256 — worth the extra minutes
on a drive that is going into a sealed payload.

The script refuses to write to the disk it is running from or to anything with
mounted partitions, and makes you type the device name to confirm. Five drives
in a row is exactly when a fast-typed device name meets a root disk one letter
away.

### From Windows or another machine

Raspberry Pi Imager → **Use custom** → pick the `.img.gz`. Then reopen the
drive (the small FAT partition, visible in Explorer) and edit
`radcam-unit.txt` — a single digit on the first line.

That file lives on the FAT partition precisely so it can be set from any OS
without mounting ext4.

---

## 3. Point the Pi at NVMe

**Do this on every new Pi.** `BOOT_ORDER` lives in the Pi's own SPI EEPROM, not
on any drive, so a perfectly flashed SSD in a Pi that still tries SD first will
boot whatever is in the SD slot — or sit at the diagnostic screen. That looks
exactly like a bad flash, and it is the most likely thing to cost an hour.

```bash
sudo tools/set-nvme-boot.sh          # sets BOOT_ORDER=0xf416
sudo tools/set-nvme-boot.sh --show   # just report, change nothing
```

`0xf416` is read right-to-left: **NVMe, then SD, then USB, then retry**. The SD
slot stays as a recovery path rather than being removed.

To do this on a Pi that will not boot at all, use Raspberry Pi Imager's
**Misc utility images → Bootloader → NVMe/USB boot** on an SD card, boot it
once, then flash the real SSD.

---

## 4. First boot

The payload provisions itself. `radcam-firstboot.service` runs once, before
`radcamd`, and:

1. generates a fresh `machine-id` and SSH host keys
2. reads `radcam-unit.txt` and sets the hostname
3. grows the root partition to fill the SSD and resizes the filesystem
4. recreates the swapfile
5. disables itself

Every step is skipped rather than retried if it looks done, and nothing in it
can block the boot: a payload with a stale hostname is a nuisance, one that
hangs waiting on provisioning is a dead camera. The log is at
`/var/log/radcam-firstboot.log`.

---

## 5. Per-unit setup, after first boot

```bash
ssh rad@radcam2

sudo radcamctl calibrate     # dosimeter: per-Pi, REQUIRED, not in the image
bash tools/verify-camera.sh  # I2C -> driver -> media graph -> capture
tools/rs422-tweak.py show    # CRC, target 0xC7, 921600, DE released
radcamctl selftest
```

The dosimeter calibration is the one thing you cannot skip. Everything else in
that list is verification.

If the camera module in this unit has never been calibrated, see the
calibration tools in `CLAUDE.md` — but note that this is per **module**, so a
module that was calibrated on another unit is already done.

---

## 6. Checklist per unit

```
[ ] SSD flashed, --verify passed
[ ] radcam-unit.txt set to this unit's number
[ ] BOOT_ORDER = 0xf416 on this Pi
[ ] first boot completed (check hostname, and df -h / shows the full SSD)
[ ] dosimeter calibrated
[ ] verify-camera.sh passes
[ ] rs422-tweak.py show: target 0xC7, DE released to hardware
[ ] hrt_initial_go OFF if this unit is going to a shared bus
```

That last one matters: `hrt_initial_go` makes the payload transmit without
waiting for a Go, which is a bench convenience on a dedicated link and a way to
talk over another experiment on a shared one.
