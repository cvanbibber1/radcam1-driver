# radcam1 — AR1335 Space Camera Driver Project

Stable project reference. Volatile status lives in `DEVELOPMENT_STATE.md` — read both
after every reboot.

> **New to this project? Read `USER_GUIDE.md` first** — it walks through
> checking the system, using the camera, calibrating, and the flight link, and
> ends with a complete command reference.
>
> **Then `HANDOFF.md`.** It covers the current
> feature set, what is blocked and why, the open decisions, and the traps that
> are expensive to rediscover.

## Mission

High-reliability, zero-intervention camera + telemetry system for a space application.
Runs unattended on a Raspberry Pi 5, booting from NVMe, power-minimised.

## Platform

| Item | Value |
|---|---|
| Board | Raspberry Pi 5 Model B Rev 1.0 (BCM2712) |
| Hostname | radcam1 |
| OS | Debian 13 (trixie), Raspberry Pi OS |
| Kernel | 6.18.34+rpt-rpi-2712 (aarch64) |
| Headers | installed at `/lib/modules/$(uname -r)/build` |
| Firmware | 2226a853 (2025/12/08) |
| libcamera | 0.7.1+rpt20260609-1 |
| rpicam-apps | 1.12.0-1 |
| Root FS | **NVMe `nvme0n1p2`** — boots from NVMe, `BOOT_ORDER=0xf416` (NVMe, then SD, then USB) |
| NVMe | `nvme0n1`, WD Green SN3000 500GB — **the live system**, 11 GB used of 457 GB |
| microSD | `mmcblk0` — now the *secondary*, holding a stale 2026-08-10 clone. Harmless: NVMe boots first, and the SD is a recovery path. |
| ISP | PiSP (`pisp_be@880000`), front end `raspberrypi,rp1-cfe` |

## Hardware map

### Camera — AR1335 (onsemi 13 MP, 4208×3120, 4-lane MIPI CSI-2)

Must work in **either** CAM/DISP port. Pi 5 22-pin FPC connector signals:

| Connector signal | CAM0 (CD0) | CAM1 (CD1) |
|---|---|---|
| I2C bus | `i2c-6` (`i2c_csi_dsi0`, `i2c@88000`) | `i2c-4` (`i2c_csi_dsi1`, `i2c@80000`) |
| I2C pins (RP1 GPIO) | SDA 38 / SCL 39 | SDA 40 / SCL 41 |
| Connector **GPIO0** (IO0) — **module FLASH output, do NOT drive** | RP1 line 34 `CD0_IO0_MICCLK` | RP1 line 46 `CD1_IO0_MICCLK` |
| Connector **GPIO1** (IO1) — **sensor ENABLE, drive HIGH** | RP1 line 35 `CD0_IO0_MICDAT0` | RP1 line 48 `CD1_IO1_MICDAT1` |
| CSI receiver | `rp1_csi0` = `csi@110000` | `rp1_csi1` = `csi@128000` |

> Stock Raspberry Pi wiring puts `cam0_reg`/`cam1_reg` on IO0 (lines 34/46).
> **This board uses IO1 instead**, so the regulator GPIOs are overridden to
> `cam0_reg_gpio=35` / `cam1_reg_gpio=48`.

GPIO controller for all of the above is `gpiochip0` (`pinctrl-rp1`, 54 lines).

### Peripherals (later phases, not yet wired up in software)

| Function | Part | Interface |
|---|---|---|
| Flight comms | ADM2582EBRWZ (UART↔RS422, isolated) | UART TX=GPIO14, RX=GPIO15 |
| Ground debug | EXTUART — mirrors flight comms | TX=GPIO24, RX=GPIO23 |
| Dosimeter | LTC2485IDDTRPBF (24-bit ΔΣ I2C ADC) | SDA=GPIO2, SCL=GPIO3 (`i2c-1`), ~2.5 mV/rad, needs one-time calibration persisted to storage |
| Illumination | TPS922051D1DSGR LED driver | PWM on GPIO18, **software-capped at 10% duty** |

> The roles are the reverse of what they were: the **NVMe is the live system**
> and the **SD card holds the stale 2026-08-10 clone**. `BOOT_ORDER=0xf416`
> tries NVMe first, so the stale copy is a fallback rather than a hazard.
> Refresh it with `tools/flash-payload-image.sh` if you want the recovery path
> to be a current system rather than an old one.

## Repository layout

```
/home/rad/driver-dev/
├── CLAUDE.md               # this file — stable reference
├── DEVELOPMENT_STATE.md    # current objective / next actions / post-reboot commands
├── kernel/ar1335/          # AR1335 V4L2 subdev kernel module
├── overlays/               # device tree overlay sources (.dts) and built .dtbo
├── libcamera/              # CamHelper + tuning JSON for the PiSP IPA
├── tools/                  # bring-up, scan and test scripts
├── docs/                   # datasheet notes, register maps
└── logs/                   # captured output from test runs
```

## Conventions

- Boot config is `/boot/firmware/config.txt`. **Always back it up** to
  `/home/rad/driver-dev/logs/config.txt.bak.<timestamp>` before editing; a bad edit
  costs a headless recovery.
- Overlays are built with `dtc -@ -I dts -O dtb` and installed to
  `/boot/firmware/overlays/`.
- Passwordless sudo is configured (`/etc/sudoers.d/claude-autonomy`).
- Reboots are expected and safe: a systemd user unit (`claude-autonomy.service`)
  runs `~/.local/bin/claude-tmux-supervisor`, which restarts this Claude session
  inside tmux session `claude-autonomy` and resumes with "continue".

## Key reference commands

```bash
# Device tree of the running system
dtc -I fs -O dts /proc/device-tree > /tmp/live.dts

# GPIO state (rpi tool pokes registers directly, ignores gpiochip ownership)
pinctrl get 35 ; pinctrl set 35 op dh

# I2C scan on the camera buses
i2cdetect -y 6    # CAM0
i2cdetect -y 4    # CAM1

# Camera enumeration
rpicam-hello --list-cameras
media-ctl -p -d /dev/media0

# Module build/load
make -C /lib/modules/$(uname -r)/build M=$PWD modules
sudo insmod ar1335.ko && dmesg | tail -30
```

## AR1335 essentials

- Chip / model ID register `0x3000` reads `0x0153`.
- Default 7-bit I2C address `0x36` (alternates `0x10`, `0x1A`, `0x3C` on some modules).
- 16-bit register addresses, 16-bit data.
- Native array 4208×3120, Bayer **GRBG**, 10-bit output.
- 4 MIPI data lanes.

> **The AR1335 has no internal oscillator**, so it needs an EXTCLK of 6–48 MHz
> before it will even acknowledge its I2C address. **This module carries its own
> 24 MHz oscillator**, which matches the register sequences, so the Pi supplies
> nothing — the 22-pin connector carries no clock anyway.
>
> **Connector IO0 is the module's flash output.** The module drives it; the Pi
> must not. Leave RP1 lines 34 (CAM0) and 46 (CAM1) as inputs.

### Key registers

| Register | Name | Use |
|---|---|---|
| `0x3000` | MODEL_ID | reads `0x0153` |
| `0x301A` | RESET_REGISTER | bit 2 = stream enable |
| `0x0202` | COARSE_INTEGRATION_TIME | exposure, in lines |
| `0x305E` | GLOBAL_GAIN | banded coarse/fine gain encoding |
| `0x0340` | FRAME_LENGTH_LINES | VTS — sets frame rate |
| `0x0342` | LINE_LENGTH_PCK | HTS |
| `0x3040` | READ_MODE | binning; bit 14 mirror, bit 15 flip |

### Register sequence provenance

onsemi's initialisation sequences are not derivable and must not be invented.
Ours come from NXP's GPL-2.0 `isp-vvcam` AR1335 driver, vendored under
`docs/reference/nxp-ar1335/` and converted by `tools/gen-regs.py` into
`kernel/ar1335/ar1335_modes.h` (928 registers across the modes). Regenerate with
`python3 tools/gen-regs.py`.

## Build and install

```bash
# Kernel module
make -C kernel/ar1335 && sudo make -C kernel/ar1335 install

# Overlay
dtc -@ -I dts -O dtb -o overlays/ar1335.dtbo overlays/ar1335-overlay.dts
sudo cp overlays/ar1335.dtbo /boot/firmware/overlays/

# Verify (bottom-up, stops at the first failing layer)
bash tools/verify-camera.sh
```

## Payload software (`radcam/`)

Python package, installed to `/opt/radcam` by `tools/install-radcam.sh`, run as
the `radcamd` systemd service with a 60 s watchdog.

| Module | Purpose |
|---|---|
| `ltc2485.py` | dosimeter ADC driver; decode ported from Analog Devices' Linduino reference |
| `dosimeter.py` | calibration, settling, volts → rad |
| `led.py` | TPS922051 PWM, **hard-capped at 10% duty** |
| `telemetry.py` | CRC-16 framed downlink, RS422 + optional mirror, TMR fields |
| `tmr.py` | triple-redundant storage with CRC + byte-wise majority vote |
| `daemon.py` | the always-on loop; every subsystem failure is contained |
| `cli.py` | `radcamctl` — status, dose, calibrate, led, selftest |

```bash
sudo bash tools/install-radcam.sh
sudo systemctl enable --now radcamd
radcamctl selftest        # exercises dosimeter, LED, UART, TMR
radcamctl dose --watch
radcamctl led 5           # 5% of full scale; >10% is clamped
journalctl -u radcamd -f
```

Config: `/etc/radcam/config.json`. Calibration: `/var/lib/radcam/dosimeter-cal.json.{0,1,2}`.

### Peripheral status

| Function | Pins | State |
|---|---|---|
| Dosimeter LTC2485 | GPIO2/3, `i2c-1`, **addr 0x24** | ✅ working (CA0/CA1 both floating) |
| LED PWM | GPIO18 → PWM0_CHAN2, `pwmchip0` ch 2 | ✅ working, capped at 10% |
| Flight RS422 | GPIO14/15 → uart0 → `/dev/ttyAMA0` | ✅ **STP/DICE at 921600, target 0xC7** |
| RS422 driver enable | GPIO4 → ADM2582E DE | ⚠️ **this board ties DE active in hardware** — see below |
| EXTUART mirror | GPIO24 TX / GPIO23 RX, **RP1 PIO** | ✅ working at **921600**, bidirectional |

> GPIO23/24 expose no TXD/RXD on a Pi 5 — only SD0/DPI/I2S/**PIO**. Userspace
> bit-banging was measured and tops out near 9600 baud, so EXTUART is driven by
> the RP1 PIO block instead: hardware bit timing, full 921600, no board change.
>
> | Tool | Purpose |
> |---|---|
> | `softuart/pio_uart` | TX only, plus a baud-accuracy self-test |
> | `softuart/pio_uart_rx` | RX, plus an internal-loopback self-test needing no jumper |
> | `softuart/pio_uart_bridge` | full duplex; stdin→wire, wire→stdout |
> | `radcam/piolink.py` | presents the bridge with the `TelemetryLink` interface |
> | `softuart/softuart` | the bit-banged version, kept as the measurement that justified PIO |
>
> Enabled with `"mirror_pio": true` in `/etc/radcam/config.json`.


## STP / DICE RS-422 — the flight interface

In flight this is the **only** channel. The payload is an *Experiment*: a slave
on a bus shared with up to five others, which **may not transmit unless spoken
to**. Specs are in `docs/stp/`; the implementation notes, measurements and the
list of remaining assumptions are in `docs/stp/README.md`.

| Packet | Direction | Size | Type |
|---|---|---:|---|
| Command | DICE → us | 120 | `0x10` |
| Command ACK | us → DICE | 8 | `0x10` |
| LRT Request | DICE → us | 14 | `0x81` |
| LRT Data | us → DICE | 1256 | `0x81` |
| HRT Flow Control | DICE → us | 14 | `0x85`/`0x86`/`0x87` |
| HRT Data | us → DICE | 1288 | `0x87` |

> **Packet type alone never identifies a packet** — `0x10`, `0x81` and `0x87`
> each mean different things by direction. Length plus direction disambiguates.

Three consequences shape the whole design:

- **The 8-byte ACK has no status field.** It means "accepted", not "done".
  Command *results* come back in LRT, keyed by the ground's `cmd_seq`.
- **All egress is polled.** Housekeeping waits for an LRT request; bulk media
  waits for `HRT Go`. Nothing is ever sent unsolicited.
- **Failure degrades, never silences.** Safe mode stops HRT but LRT keeps
  answering, because a quiet payload is indistinguishable from a dead one.

### Byte order — the trap

**ICD structures are big-endian** (envelope, command header, LRT, HRT), and the
CRC is always the **final two bytes of every message**.
**Command args and response blobs are little-endian**, because they are the
existing `radcam.protocol` format, dispatched unchanged. Every `struct` call in
`radcam/stp/experiment.py` is little-endian; everywhere else in `radcam/stp/`
is big-endian.

### Storage slots

Captures go into **numbered slots** (16 by default), not auto-incrementing
media ids, so a canned hex command addresses the same place every pass.
`SLOT_CAPTURE_IMAGE`, `SLOT_RECORD_START` (held on the Pi, not streamed),
`SLOT_DOWNLOAD`, `SLOT_DELETE`. Deletion is what frees space. Each slot carries
a CRC-32 checked on read; the index is TMR-protected.

### Commands as hex strings

The ground station is specified in `docs/host/HOST_APP_SPEC.md`, with
`docs/host/HOST_APP_UPDATE_SPEC.md` carrying the multi-camera and
configurable-CRC delta (including an LRT layout fix the base spec gets wrong).

`radcam/stp/catalogue.py` holds the command set as data; `tools/stp-command.py`
renders pasteable 120-byte hex strings and `--verify` proves all 39 are
accepted by a real decoder. Canned strings set the force flag, or a second
paste would be suppressed as a retransmission.

### Channel roles

| Channel | Carries | Never carries |
|---|---|---|
| **LRT** | telemetry and vitals only | bulk data of any kind |
| **HRT** | live video, and chunked file transfer | housekeeping |

### Live video

H.264 over HRT, **640x480 at 15 fps and 600 kbit/s by default**, with the crop
box centred on any sensor pixel — crop equal to output gives 1:1 native pixels.
Measured end to end: 15.0 fps, 584 kbit/s, keyframe per second, zero chunk CRC
failures, decodes with no errors.

`rpicam-vid --codec h264` **does not work on a Pi 5** — no hardware encoder, no
`/dev/video11`, and rpicam-apps built without libav. The pipeline is
`rpicam-vid --codec yuv420 | ffmpeg -c:v libx264`, run with
`sliced-threads=0:threads=1`.

`hrt_initial_go` and `stream_autostart` bring the payload up already
transmitting live video, with no `HRT Go` and no `STREAM_START`. Both are bench
conveniences for a dedicated link — **`hrt_initial_go` must be off in flight**,
because a slave that transmits before being asked talks over whichever
experiment DICE is listening to. A `Stop` still closes a tap that opened itself.

**A stream drops, it never queues.** Bounded ring, oldest discarded, flushed
when HRT closes — buffering would turn a bandwidth shortfall into unbounded
latency. Live video preempts file transfer; file chunks use the gaps between
frames.

### Error correction

HRT **file transfer** carries per-chunk CRC-32 (detection) plus XOR parity
every `fec_group_size` chunks (correction): one lost chunk per group is rebuilt
with no retransmission, at 6.25% overhead at the default of 16. Parity is
emitted as each group closes, so an interrupted transfer still leaves completed
groups repairable. Beyond one loss per group, explicit RESEND takes over.
Live video carries no parity — a late frame is worthless, and H.264 recovers at
the next keyframe for free.

### DE: check how the board wires it before trusting the software

This board pulls **DE up** and **/RE down**, so the ADM2582E is meant to sit
permanently enabled in full duplex with the payload as the only transmitter.
Software driving GPIO4 low between packets **held the transmitter disabled**,
because a push-pull output beats a pull-up.

That failure is invisible from the payload: DE toggles, the UART counts every
byte transmitted, and TEMT confirms the shift register emptied. None of that
proves the differential driver was enabled in time for those bits to reach the
wire.

Set `"de_control": false` in the `stp` config when DE is tied active. The link
then turns GPIO4 back into an **input** at startup rather than merely refraining
from asserting it — refraining is not enough, the pin has to stop being driven.
Confirm with `pinctrl get 4`, which should read `ip pu | hi`.

Software DE timing below applies only when `de_control` is true.

### DE timing — measured, not assumed

DE is on **GPIO4**, active high. `tcdrain()` is unusable: it holds the line
8–13 ms regardless of size (140× the wire time of an 8-byte ACK), which jams
other experiments. The transmitter uses `TIOCSERGETLSR`/`TIOCSER_TEMT` instead
and releases within **~30 µs at any packet size**, sustaining 89 kB/s of HRT
payload at 99.7% wire utilisation. Re-verify with `tools/stp-de-timing.py`.

Wire settings — CRC, target id, coverage window, DE, cameras — are changed
with `tools/rs422-tweak.py` and documented in **`docs/rs422tweak.md`**, which
is also the pre-test checklist. The CRC is fully expressible in configuration
(named variant or explicit poly/init/reflection/xor/store), so a disagreement
with the flight computer is a config change, never a rewrite.

```bash
tools/rs422-tweak.py show      # every wire setting, as the daemon reads it
tools/rs422-tweak.py sample    # real packets, for the ground to check against
tools/stp-verify.py            # 85 reliability / safety / autonomy checks
tools/stp-sim.py --self-test   # 21-check protocol conversation, no hardware
tools/stp-de-timing.py --throughput
tools/stp-crc-solve.py --bin capture.bin   # recover the real CRC parameters
```

Enabled by `"stp": {"enabled": true}` in `/etc/radcam/config.json`. That
replaces the flight telemetry port with a null link, so the ASCII beacon
**cannot** reach the DICE bus, and disables the old COBS command server.

## Flashing more payloads

Five more cameras get built from one image; see **`docs/FLASHING.md`**.

```bash
sudo tools/build-payload-image.sh                     # ~1.2 GB .img.gz
sudo tools/verify-payload-image.sh IMAGE              # 23 checks
sudo tools/flash-payload-image.sh IMAGE /dev/sdX --unit 2 --verify
sudo tools/set-nvme-boot.sh                           # on each new Pi
```

The image is a clone of this system with identity and per-Pi state stripped,
and `provisioning/radcam-firstboot.sh` regenerates machine-id and SSH host
keys, sets the hostname from `radcam-unit.txt` on the boot partition, and
expands the root filesystem to fill the SSD.

Two asymmetries worth remembering, because they are easy to get backwards:

- **Camera colour calibration travels with the camera module** (it lives on the
  module's own EEPROM), so a calibrated module carries its numbers to any unit.
- **Dosimeter calibration belongs to the Pi** (the LTC2485 is on the Pi board),
  so it is excluded from the image and every unit needs its own
  `radcamctl calibrate`. A cloned dose calibration is a confidently wrong
  number, which is worse than none.

`BOOT_ORDER` lives in the Pi's SPI EEPROM and no disk image can carry it — a
perfect SSD in a Pi that still tries SD first looks exactly like a bad flash.

## libcamera

libcamera will not touch a sensor it has no CamHelper for. Ours lives in
`libcamera/` and is built into a local libcamera install:

| File | Purpose |
|---|---|
| `libcamera/cam_helper_ar1335.cpp` | gain/exposure mapping; registered as `ar1335` |
| `libcamera/ar1335.json` | PiSP tuning (AWB/AGC/CCM) - **starting point, not calibrated** |
| `libcamera/README.md` | build, install, revert, and what still needs measuring |

Built from `github.com/raspberrypi/libcamera` into `/usr/local`, which shadows
the Debian package. Revert with
`sudo rm -rf /usr/local/lib/aarch64-linux-gnu/libcamera* && sudo ldconfig`.

## Camera calibration

Per-camera calibration lives on the **camera module's own EEPROM** (24C64, 8 KB,
0x50 on the camera I2C bus), not on the Pi - cameras and Pis are interchangeable,
so calibration has to travel with the module. Three redundant copies with CRC32,
self-repairing on read, same approach as the dosimeter baseline.

| Command | Purpose |
|---|---|
| `tools/focus.py [--preview\|--http PORT]` | live focus meter; terminal image or MJPEG over SSH |
| `tools/chart-framing.py` | scores chart framing/focus before calibrating |
| `tools/calibrate-camera.py --camera-id X --apply` | **the colour balance command**: capture, find the chart, solve, write EEPROM, install to the live tuning |
| `tools/calibrate-camera.py --from-capture F.jpg` | solve from a saved frame (needs its `.dng` alongside) |
| `tools/calibrate-camera.py --centres F.json` | supply patch centres by hand when detection fails |
| `tools/calibrate-camera.py --show \| --erase` | inspect or clear a camera's calibration |

Glare, white balance and the matrix are solved **together**
(`radcam.ccm.calibrate_from_patches`), because they are not independent.
Veiling glare is additive, so no gain or matrix downstream can remove it; left
in, it is what makes dark saturated patches read as washed-out greys and it
bends both of the other two as they try to absorb it. The black patch measures
it, but only where the black patch sits, so it sets the scale of a search
rather than being subtracted outright.

The matrix is judged on whether it *helps*, not against a fixed dE. A fit that
takes dE 22 to 7 is a large visible win even though 7 is not a good absolute
number; a matrix fitted to patches the capture could not measure barely moves
the error at all, and that is the case worth refusing.
| `tools/find-crop.py --store` | detect the usable frame region, store on the camera |
| `tools/measure-awb.py` | raw R/G, B/G from a grey target |
| `tools/fit-awb-curve.py --anchor\|--points` | build the AWB colour-temperature curve |
| `tools/calib-box-spec.py --dfov D` | dimension the calibration fixture from a measured FOV |
| `tools/calibrate-shading.py --store --apply` | lens shading / vignetting; needs a **flat field** |
| `tools/calibrate-distortion.py --store` | radial distortion from straight edges in any scene |
| `tools/calibrate-distortion.py --undistort IN OUT` | correct an image with the stored model |
| `tools/calibrate-response.py --store` | exposure & gain linearity, saturation point |

### Everything is stored on the camera, and applied at boot

All calibration lives in one JSON record on the module's EEPROM, merged section
by section (`CameraEEPROM.update()`), so each tool adds its own results without
disturbing the others:

| Section | Written by | Applied to |
|---|---|---|
| `awb`, `ccm`, `black_level` | `calibrate-camera.py` | `rpi.awb`, `rpi.ccm`, `rpi.black_level` |
| `shading` | `calibrate-shading.py` | `rpi.alsc` |
| `distortion` | `calibrate-distortion.py` | post-process (no ISP support) |
| `response` | `calibrate-response.py` | reference data |

`radcam-calibration.service` runs `python3 -m radcam.calibration --any-bus`
before `radcamd` and before anything opens the camera, so a module carries its
calibration to any Pi with no manual step. Verified by wiping the tuning to
identity and confirming the service restores it.

Shading and distortion are stored as **models, not grids**: libcamera wants
three 32x32 tables and the EEPROM has ~2 KB for everything, so twelve
coefficients are stored and the 3072-entry grid is regenerated on apply.

The calibration fixture itself - a light-tight printed box holding the camera
and every target - is specified in `docs/calibration-box.md`. Measure the FOV
before building it; every dimension scales with that number.

`radcam/calibration.py` reads the EEPROM at startup and applies black level,
AWB curve and CCM to the tuning file, so any Pi picks up the right values for
whichever module it is looking at.

## Tests

Stdlib `unittest` only — no pytest, no extra dependency on the flight Pi.

```bash
python3 -m unittest discover -s tests -t .      # 217 tests, ~19 s
```

| File | Covers |
|---|---|
| `tests/test_eeprom.py` | framing, CRC, majority vote, repair, section merge |
| `tests/test_calibration_math.py` | shading / distortion / colour solvers against known answers |
| `tests/test_protocol.py` | region capture bounds, EEPROM commands, write protection, clamping |
| `tests/test_stp_packets.py` | ICD packet sizes/offsets, CRC coverage, resync on a shared bus |
| `tests/test_stp_experiment.py` | ACK/LRT/HRT state machine, dedup, safe mode, TMR |
| `tests/test_stp_fec.py` | parity recovery, loss and resend |
| `tests/test_stp_stream.py` | stream config clamping, frame parsing, drop-not-delay |
| `tests/test_slots.py` | slot lifecycle, CRC checks, command hex strings |

`tests/support.py` has `FakeEEPROM`, which subclasses `CameraEEPROM` at the raw
read/write boundary so framing, CRC and repair are exercised as the real code,
and emulates the 24C64's page-wrap so a driver that stopped chunking would fail
rather than pass quietly.

## Tooling

| Script | Purpose |
|---|---|
| `tools/scan-camera.sh` | assert enables, scan both camera I2C buses |
| `tools/sweep-enable.sh` | try every IO0/IO1 combination on both ports |
| `tools/csi-presence.sh` | I2C pull-up probe — **inconclusive**, see notes |
| `tools/verify-camera.sh` | full stack check: I2C → driver → media graph → capture |
| `tools/gen-regs.py` | regenerate the mode register tables |
