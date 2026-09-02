# rs422tweak — changing the wire settings

This is the guide for the settings that decide whether the flight computer
**accepts** our packets: the CRC, the target id, the CRC coverage window, the
byte order, the baud rate and DE. It also covers adding or changing commands.

> **Why this file exists.** Every one of these settings fails silently. If the
> CRC is wrong we transmit perfectly formed packets at the right baud rate with
> the right framing, our own logs show every byte going out, and the other end
> discards all of it. There is no error to read. So the settings have to be
> fast to change and fast to read back, and that is what `tools/rs422-tweak.py`
> is for.

Everything lives in the `stp` block of `/etc/radcam/config.json`. Nothing here
requires editing Python.

---

## 1. The tool

```bash
tools/rs422-tweak.py show          # every wire setting, as the daemon reads it
tools/rs422-tweak.py diff          # only what differs from the shipped defaults
tools/rs422-tweak.py list-crc      # every CRC variant, by name
tools/rs422-tweak.py sample        # real packets built with these settings
tools/rs422-tweak.py crc 01 02 03  # CRC of some bytes, under these settings
tools/rs422-tweak.py set KEY=VALUE [...] [--restart] [--dry-run]
```

`set` backs the config up to `logs/config.json.bak.<timestamp>`, validates the
**result** before writing, re-parses the written file, and only then restarts.
A rejected value changes nothing. Writing needs `sudo`; reading does not.

### Current setting

```
CRC-16/CCITT-FALSE: poly=0x1021 init=0xFFFF refin=False refout=False
xorout=0x0000 store=big-endian, covers packet[4:crc]
target 0xC7, 921600 baud, big-endian structures, DE released to hardware
```

This is verified against the flight computer's own table
(`tests/test_stp_crc_flight.py`), so change it only if the ground tells you the
far end disagrees.

---

## 2. Changing the CRC

### By name

```bash
sudo tools/rs422-tweak.py set crc=CRC-16/XMODEM --restart
```

Names are matched tolerantly: `CRC-16/XMODEM`, `xmodem` and `XMODEM` are the
same request. `list-crc` prints all fifteen. Naming a standard variant **drops
any leftover custom overrides**, so you cannot half-switch by accident.

### By parameters

Anything the catalogue does not cover is expressible directly. The base is
CCITT-FALSE, and you override only what differs:

```bash
sudo tools/rs422-tweak.py set crc=custom \
    crc_poly=0x8005 crc_init=0 \
    crc_reflect_in=true crc_reflect_out=true --restart
```

| Key | Meaning | Accepts |
|---|---|---|
| `crc_poly` | generator polynomial | `0x1021`, `"0x1021"`, `"1021h"`, `4129` |
| `crc_init` | initial register value | as above |
| `crc_reflect_in` | reflect each input byte | true / false |
| `crc_reflect_out` | reflect the final register | true / false |
| `crc_xor_out` | final XOR | as `crc_poly` |
| `crc_store` | how the 16-bit result is written | `big` / `little` |
| `crc_start` | first byte the CRC covers | integer, default 4 |

JSON has no hex literal, so hex is accepted as a string. This matters: someone
reading a datasheet writes `0x1021`, and a config that silently read that as a
decimal 1021 would be wrong in a way nobody would spot.

`crc_store` is a **separate question from the algorithm** — a CRC-16/CCITT-FALSE
stored little-endian is still CCITT-FALSE, and mixing the two up is a common
first-contact failure. The stored order is the last two bytes of the packet.

`crc_start` is where coverage begins. The ICD states 4 (just past the sync
word) for the HRT classes and does not confirm it for the others; if the far end
covers the sync word too, set `crc_start=0`.

If a custom set happens to reproduce a standard variant, the tool says so:

```
resulting CRC: CUSTOM (CRC-16/ARC): poly=0x8005 init=0x0000 ...
```

### If a typo gets through

A bad CRC name does **not** stop the payload coming up. It logs a complaint and
falls back to CCITT-FALSE, because a payload answering with the wrong checksum
is diagnosable from the ground while a payload that never answers is
indistinguishable from dead hardware. Check the log after any change:

```bash
journalctl -u radcamd --since -1m | grep -i crc
```

### If you don't know what the far end uses

Capture its traffic and solve for it, rather than guessing:

```bash
tools/stp-crc-solve.py --bin /var/log/radcam/rx.bin
```

It searches every variant, both stored byte orders and several coverage windows,
and prints the `set` line for whatever survives all the samples. Supply at least
three packets — one sample admits coincidental matches.

---

## 3. The other wire settings

```bash
sudo tools/rs422-tweak.py set target=0xC7 --restart
sudo tools/rs422-tweak.py set baud=921600 --restart
sudo tools/rs422-tweak.py set big_endian=true --restart
sudo tools/rs422-tweak.py set lrt_trailer=crc --restart
```

| Key | Default | Notes |
|---|---|---|
| `target` / `target_id` | `0xC7` | **Mission-assigned. Constant for the entire chain, including the thermal camera.** Packets with any other target id are dropped before application code sees them. |
| `baud` | `921600` | |
| `port` | `/dev/ttyAMA0` | |
| `big_endian` | `true` | ICD **structures**. Command args and response blobs stay little-endian regardless — see §5. |
| `lrt_trailer` | `crc` | `zero` exists only for testing against a non-conforming peer. |
| `de_control` | `false` | **See the warning below.** |
| `de_gpio` | `4` | |
| `log_rx` | `true` | raw receive capture to `/var/log/radcam/rx.bin` |

### ⚠️ `de_control` — the setting that silently kills the transmitter

This board pulls **DE up** and **/RE down**: the ADM2582E is meant to sit
permanently enabled. Software driving GPIO4 low between packets **held the
transmitter off**, because a push-pull output beats a pull-up. With
`de_control: false` the link turns GPIO4 back into an *input* at startup —
refraining from asserting it is not enough, the pin has to stop being driven.

**This has silently reverted to `true` once already.** Check it before any test:

```bash
tools/rs422-tweak.py show | grep DE      # want: released to hardware
pinctrl get 4                            # want: 4: ip pu | hi
```

---

## 4. Proving a change before trusting the link

`sample` builds real packets with the current settings, so a disagreement with
the flight computer can be found by inspection instead of on a live link:

```
Command ACK      8 bytes  1acffc1d10c7b7d7
                     CRC over [4:6] = b7d7
LRT Data      1256 bytes  head 1acffc1d81c700000000000000000000 ... crc befb
HRT Data      1288 bytes  head 1acffc1d87c700000000000000000000 ... crc eb2f
```

Send those three lines to the ground and have them run the far end's own CRC
over the same bytes. If the values match, the CRC is not your problem.

Then the offline checks, neither of which needs hardware:

```bash
tools/stp-sim.py --self-test    # 21-check protocol conversation
tools/stp-verify.py             # 85 reliability / safety / autonomy checks
python3 -m unittest discover -s tests -t .
```

---

## 5. Changing or adding commands

Commands are **data**, in `radcam/stp/catalogue.py`. Rendering, verification and
hex generation all read that table, so a new command is one entry plus one
handler — never a change to the packet layer.

```bash
tools/stp-command.py --list                  # all 41, by group
tools/stp-command.py SLOT_CAPTURE_IMAGE slot=3
tools/stp-command.py --verify                # every command through a real decoder
```

`stp-command.py` reads the CRC, byte order and coverage window from
`/etc/radcam/config.json` (override with `--config`), so the hex it prints
always matches what the payload will accept. **Regenerate every canned string
after changing the CRC** — a canned string carries its own CRC baked in, and a
stale one is rejected silently.

The output is a pasteable 120-byte hex string. Canned strings set the **force
flag**, because a second paste of the same string would otherwise be suppressed
as a retransmission.

### Adding one

1. **Pick an opcode** in `StpOp` (`radcam/stp/experiment.py`). The range is
   `0x64–0x7B`; `0x74–0x76` and `0x7C–0x7E` are reserved for the thermal
   camera by `docs/stp/MULTI_CAMERA_SPEC.md`. Opcodes must not collide with the
   legacy `radcam.protocol.Msg` set.
2. **Add a `CommandSpec`** to `CATALOGUE` with its name, opcode, one-line
   summary, group, and its argument `Field`s. Argument formats are **struct
   format characters, little-endian** — that is the existing
   `radcam.protocol` blob format and it is dispatched unchanged.
3. **Handle it** in `Experiment._dispatch_stp` alongside the neighbouring
   opcodes, and add it to the safe-mode allow-list only if it is safe to run
   with the payload degraded.
4. **Run `tools/stp-command.py --verify`** — it proves every catalogue entry is
   accepted by a real decoder, so a wrong field width is caught here rather
   than in a pass.

### The byte-order trap

**ICD structures are big-endian** (envelope, command header, LRT, HRT) and the
CRC is always the final two bytes. **Command arguments and response blobs are
little-endian.** Every `struct` call in `radcam/stp/experiment.py` is
little-endian; everywhere else in `radcam/stp/` is big-endian. This is a
deliberate convention, not an inconsistency to tidy up — the blob format
predates the ICD and is dispatched to existing code untouched.

---

## 6. Cameras

Configured at the **top level** of the config, not in the `stp` block:

```json
"cameras": [
  {"index": 0, "gpio": 48, "name": "AR1335", "i2c_bus": 4, "always_on": true}
],
"default_camera": 0
```

Exactly one camera may be enabled at a time — two sensors sharing an I2C address
answer together, and two driving the same CSI lanes contend. `SELECT_CAMERA`
(`0x6D`) disables everything and then enables one; there is no "enable camera N".

**`always_on: true`** says the enable line is not ours to switch. On this board
the AR1335 sits on the CAM1/CD1 connector and its enable (RP1 48) is held by the
kernel as `cam1_reg`, so it is powered at boot. The selector therefore never
claims that GPIO — claiming it would let us power down the only camera in the
payload — and reports it as the active camera, because it is on whatever we do.
Selecting any *other* camera while an always-on camera exists is **refused**,
since the always-on one cannot be turned off and the result would be exactly the
contention the interlock prevents.

Adding the thermal camera therefore means giving **both** cameras switchable
GPIOs and setting `always_on: false` on both. Free lines: 5–13, 16, 17, 19–22,
25–27. See `docs/stp/MULTI_CAMERA_SPEC.md`.

```bash
tools/rs422-tweak.py show | tail -5     # cameras and which is active
journalctl -u radcamd --since -1m | grep -i camera
```

---

## 7. Pre-test checklist

```bash
tools/rs422-tweak.py show               # CRC, target 0xC7, 921600, DE released
pinctrl get 4                           # 4: ip pu | hi
systemctl is-active radcamd             # active
journalctl -u radcamd --since -2m | grep -Ei "crc|camera|RS-422|DE on"
tools/stp-sim.py --self-test            # ALL PASS
rpicam-still --immediate -n -t 2000 -o /tmp/check.jpg   # the camera images
```

**Any config change needs `sudo bash tools/install-radcam.sh` if you changed
Python** — the service runs from `/opt/radcam`, not from this repo, and editing
the repo alone changes nothing about what is running.
