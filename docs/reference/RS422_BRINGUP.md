# RS-422: wiring it correctly, and getting the first bits

A handoff note for another engineer or AI bringing up an RS-422 link. Written
after debugging one to a working state on an STM32F412 + ADM2582E camera, and
aimed at the case where *the same cable that worked on one device produces
nothing, or garbage, on another*.

Everything marked **[verified]** was measured on real hardware. Everything else
is standard RS-422 practice.

---

## 1. The one thing that causes most of the pain

**RS-422 pin naming is not standardised, and vendors contradict each other.**

You will see all of these for the same four wires:

| Seen on | Driver (output) | Receiver (input) |
|---|---|---|
| ADM2582E, MAX490, many transceivers | `Y`, `Z` | `A`, `B` |
| Most USB converters and terminal blocks | `TX+`, `TX-` | `RX+`, `RX-` |
| Some cameras and PLCs | `A`, `B` | `Y`, `Z` |
| TIA/EIA-422 itself | `A`, `B` | `A'`, `B'` |

Note the third row. A device labelled `A`/`B` may be presenting its **driver**,
not its receiver — the exact opposite of the transceiver in row one. Two
devices both labelled `A`/`B` can be an output-to-output pair that will never
work.

**Never wire by letter. Wire by function.** Establish, for each end
independently, which pair is the output and which is the input, then cross
them. If a datasheet is ambiguous, measure (section 4).

> This is the most likely explanation for "same wires, different camera,
> no output". Your thermal camera and your visual camera very likely use
> different letter conventions for the same physical roles.

---

## 2. What RS-422 actually is

- **Differential.** Each signal is a voltage *difference* across a twisted
  pair, typically ±2 V or more when driven. Neither wire is meaningful alone.
- **Full duplex, four wires**, in two independent pairs. One pair carries
  device→host, the other host→device. They are not interchangeable and never
  touch each other.
- **One driver per pair.** RS-422 is point-to-multipoint: one talker, up to ten
  listeners. If two devices can drive the same pair, you want RS-485, and you
  must manage driver-enable (section 5).
- **It is only a physical layer.** It says nothing about baud, framing, or
  protocol. Almost always UART framing rides on top, so both ends must agree on
  baud, data bits, parity and stop bits as well as on wiring.

Do not confuse it with RS-485, which is commonly two wires, half duplex, and
shares one pair for both directions. Some cameras marked "RS-422" are wired as
2-wire half duplex. If your device has only two signal terminals plus ground,
you are dealing with the half-duplex case and the crossover rules below do not
apply — you simply connect `+` to `+` and `-` to `-`, and the *timing* of who
talks becomes the problem instead.

---

## 3. Wiring rules

### Cross the pairs

Output of one end goes to input of the other, both ways.

```
   Device                          Host / converter
   ------                          ----------------
   driver +   (Y / TX+ / A)  --->  receiver +  (RX+ / A)
   driver -   (Z / TX- / B)  --->  receiver -  (RX- / B)
   receiver + (A / RX+)      <---  driver +    (TX+ / Y)
   receiver - (B / RX-)      <---  driver -    (TX- / Z)
   GND                       ----  GND
```

**[verified]** On the project this note came from, the working mapping was:

| Camera (ADM2582E) | USB converter |
|---|---|
| `A`, `B` — receiver inputs | `TX+`, `TX-` |
| `Y`, `Z` — driver outputs | `RX+`, `RX-` |

The fault that cost the most time was landing the camera's `A`/`B` on the
converter's `RX+`/`RX-`. Both ends were then listening on that pair and neither
was talking. The camera transmitted perfectly the whole time, which made it
look like a receive-side firmware bug rather than a wiring error.

### Ground

Join the grounds unless the link is galvanically isolated. Differential does
not mean ground-free: the receiver still needs both inputs inside its
common-mode range, typically −7 V to +12 V. A floating pair drifts out of it.

**[verified]** An unterminated, undriven pair on this board measured **5 V DC**
across A and B — well outside anything a receiver will interpret sensibly.

### Termination

Fit **120 Ω across the receiver inputs at the far end of each pair**, matching
the cable's characteristic impedance. One resistor per pair, at the receiving
end only, not at both ends of a point-to-point link and not in the middle.

**[verified]** Removing the 120 Ω across the camera's `A`/`B` made things
strictly worse, not better. If a link is marginal, adding correct termination
is a candidate fix; removing it almost never is.

For very short bench cables (well under a metre) termination matters little,
but it costs nothing to have it right.

### Polarity within a pair

Getting `+` and `-` backwards inverts the data. The signature is distinctive
and is covered in section 7: you get **bytes, but wrong ones**, usually with
framing errors — not silence.

---

## 4. Determining which pair is which, empirically

When the labels cannot be trusted, a multimeter settles it in a minute.

1. **Power the device, leave the link disconnected.** Nothing else attached.
2. **Find the driver pair.** An enabled, idle RS-422 driver holds its outputs
   at a steady differential of roughly 2–5 V, one wire high and the other low
   relative to ground. Measure across candidate pairs; the driver pair shows a
   clear, stable differential.
3. **Confirm by making it talk.** If the device transmits on its own, or you
   can make it, the driver pair's differential will visibly move — on a scope,
   a burst; on a meter, a twitching average.
4. **The receiver pair** shows no drive of its own. It may sit near 0 V, or be
   pulled to a bias by fail-safe resistors, or float at some arbitrary voltage
   if unterminated and unbiased.
5. **Polarity:** with the line idle, UART idle state is *mark* (logic 1). On a
   correctly-connected receiver, `A`/`RX+` is **negative** with respect to
   `B`/`RX-` in the idle state under the common convention — but conventions
   vary, so treat this as a starting guess and be ready to swap (section 7).

A scope beats a meter here. If you have one, probe the driver pair while the
device transmits and read the bit period directly: **baud = 1 / bit period**.
That resolves an unknown baud rate without guessing.

---

## 5. Driver enable and receiver enable

Transceivers usually expose:

- **`DE`** — driver enable, active high. When low the driver is high-impedance.
- **`RE`** (usually `RE̅`, active low) — receiver enable. When low the receiver
  is on.

For a **point-to-point** link with one talker per pair, the simple and robust
choice is to leave the driver permanently enabled and the receiver permanently
enabled. Nothing contends.

For a **shared pair** (multidrop, or half-duplex RS-485), the driver must be
raised only while transmitting and dropped immediately after the last bit has
left the shift register — not when the DMA completes, which is earlier. Getting
this wrong jams the bus for everyone else and is a common cause of "it works
alone but not with the others attached".

**[verified]** On this board `RE̅` is tied to ground (receiver always on) and
`DE` has a pull-up fitted (driver always on). The firmware still contains the
per-packet turnaround behind a compile-time switch, currently off, because only
one device is on the pair. Note that **holding the driver on does not prevent
reception in a 4-wire link** — the pairs are independent. That surprises people
and leads them to disable a driver that never needed disabling.

---

## 6. Baud rate: pick one both ends can actually generate

Both ends divide a clock. If either cannot hit the requested rate exactly, the
error accumulates across a character and the far end samples in the wrong
place. **Total error budget across both ends is roughly 2–3%**; beyond that you
get framing errors and corrupt bytes.

**[verified] STM32 side.** USART2 was clocked from APB1 at 50 MHz and divides
by `BRR/16`, so only some rates land exactly:

| Baud | Divisor | Error |
|---:|---|---|
| 921,600 | 54.25 | 0.5% |
| 1,000,000 | 50 | exact |
| 1,500,000 | 33.33 | 1.0% |
| 2,000,000 | 25 | exact |
| 3,000,000 | 16.67 | **2.0% — too far** |

Ceiling is PCLK/16 = 3.125 Mbaud with 16× oversampling.

**[verified] FTDI side.** FT232R generates `3,000,000 / (n + f)`, so 3 Mbaud
and 2 Mbaud are exact special cases and 1 Mbaud is 3M/3. 2 Mbaud was chosen for
high-rate testing precisely because **both ends hit it exactly**.

If a device's baud is unknown, measure the bit period on a scope rather than
sweeping blindly — but if you must sweep, remember the discriminator in the
next section: a wrong baud still produces *bytes*.

Also agree on framing. `8N1` is the overwhelming default, but a device
expecting `8E1` or two stop bits will produce framing errors that look exactly
like a baud mismatch.

---

## 7. Failure signatures — the fastest way to localise a fault

This table is the most useful thing in this note. **The distinction between
"no bytes at all" and "wrong bytes" separates wiring faults from
configuration faults**, and it is what finally localised the fault on this
project.

| Symptom | Almost certainly |
|---|---|
| **Zero bytes received, ever.** No framing errors, no corrupt bytes, nothing. | Wiring. Pairs swapped, an open conductor, wrong terminals, device not powered, or the driver disabled. A wrong *baud* cannot produce this. |
| **Bytes arrive but are garbage**, framing errors, lots of `0x00`/`0xFF` | Baud mismatch, framing mismatch, or **inverted polarity within a pair**. Try swapping the two wires of that one pair first — it is free and it is often the answer. |
| **Correct at low baud, corrupt at high baud** | Signal integrity: missing or wrong termination, unshielded or untwisted cable, excessive length, or a baud neither end hits exactly. |
| **Occasional corrupt packets, most fine** | Marginal integrity, or the *host* is dropping bytes (section 8). Check the host first; it is more often the culprit than the cable. |
| **Device transmits fine but ignores everything sent to it** | Its receive pair. This was the actual fault here. Its transmit path working tells you nothing about its receive path — they are physically separate. |
| **Works alone, fails with other devices attached** | Driver-enable contention, or termination now wrong for the topology. |
| **Worked on device A, silent on device B, same cable** | Pin-naming convention differs between the devices. See section 1. Try swapping the two pairs. |

---

## 8. Host-side traps that masquerade as link faults

Two cost real time on this project.

**[verified] The receive buffer is too small.** If your host does any
appreciable work between reads — decoding, decompressing, rendering — the
driver's default receive buffer overflows and you lose bytes. This appears as
CRC errors and truncated frames and looks exactly like a noisy cable. On
Windows with pyserial:

```python
port = serial.Serial(name, baud, timeout=0.2)
port.set_buffer_size(rx_size=1 << 20)   # 1 MB
```

Symptom before the fix: a display running at half the rate of a command-line
tool reading the same stream, with CRC errors that vanished when the buffer was
enlarged.

**A stale process holds the port.** One process owning a COM port silently
starves every other. If a tool that worked yesterday sees nothing today, list
processes holding the port before touching the wiring.

**FTDI latency timer.** Default 16 ms. It does not lose data, but it batches
arrivals, so any timing you infer from arrival timestamps is meaningless at
that resolution. Reduce it in the driver properties if you need timing.

---

## 9. Getting the first bits: a bring-up ladder

Run these **in order**. Each step is decisive: it either passes or tells you
where to look, and passing rules out everything below it. Do not skip ahead.

### Step 0 — Establish the facts on paper

Write down, for both ends: which pair is driver, which is receiver, baud,
framing, and whether termination is fitted. If you cannot fill in every cell,
measure it (section 4). Most failed bring-ups are a guess in one of these
cells.

### Step 1 — Prove the host converter works, with no device attached

Loop the converter back on itself: jumper its `TX+` to its `RX+` and `TX-` to
its `RX-`. Send a distinctive marker and check it returns.

```python
import serial
p = serial.Serial("COM34", 921600, timeout=0.5)
p.write(b"\xA5\x5ALOOPBACK\x5A\xA5"); p.flush()
print(p.read(64))
```

- **Marker returns** → converter, driver, cable to the terminal block and your
  host software are all good. The fault is at the device. Continue.
- **Nothing** → the fault is the converter, its wiring, or its port. Fix this
  before touching the device. Note that a 2-wire (half-duplex) converter will
  fail this test by design.

### Step 2 — Listen to the device

Attach only the device→host pair. Power the device. Do not send anything.

```python
import serial, time
p = serial.Serial("COM34", 921600, timeout=0.2)
end, total = time.time() + 5, 0
while time.time() < end:
    total += len(p.read(4096))
print(total, "bytes in 5 s")
```

- **Bytes, and they look like plausible protocol** → device→host works. Go to
  step 3.
- **Bytes, but garbage** → baud, framing, or pair polarity. Swap that pair's two
  wires; if still wrong, sweep baud or measure it on a scope.
- **Zero bytes** → wiring, power, or the device genuinely transmits nothing
  unsolicited. **Check that assumption before rewiring**, see the warning
  below.

> ### The trap that will cost you an afternoon
>
> **[verified]** Many devices — including the camera this note came from —
> transmit *nothing at all* until addressed. A passive listener sees perfect
> silence from a perfectly working device.
>
> Before concluding the link is dead, determine from the device's
> documentation whether it free-runs or is strictly request-response. If it is
> request-response, step 2 cannot pass and you must go straight to step 3.
>
> Firmware on the project deliberately returns **0 bytes** to a passive
> listener; that is correct behaviour, not a fault.

### Step 3 — Talk to the device

Attach the host→device pair. Send whatever minimal, well-formed request the
device documents — a status query, an identify, a ping. Watch for any reply.

- **Reply** → the link is up in both directions. Done.
- **No reply, and the device has receive counters** → read them. This is the
  single most valuable diagnostic a device can offer:
  - **Counters at zero, including error counters** → nothing is reaching the
    receiver at all. Wiring on the host→device pair.
  - **Error counters incrementing** → bytes *are* arriving and being rejected.
    Baud, framing, polarity, address/target-ID mismatch, or checksum
    convention. The physical link is fine; the problem is above it.
- **No reply, no counters** → probe the device's receiver inputs with a scope
  while transmitting. Signal present means the fault is inside the device or
  in its configuration; no signal means it is the wiring.

### Step 4 — Prove it under load

Only now stream continuously for minutes and count errors. A link that passes
steps 1–3 can still fail here on termination, cable quality, or host buffering
(section 8).

**[verified]** Reference figures from the working link: at 921,600 baud,
~79,000 bytes/s sustained with **zero CRC errors over 180 seconds**. At 2 Mbaud
on the same bench wiring, 2 CRC errors per ~4,400 packets in two runs of three
— an example of a link that passes every functional test but is measurably less
clean, and a reason to prefer the slower rate when margin matters more than
throughput.

---

## 10. Measuring rates without fooling yourself

**[verified]** A trap worth knowing, because it produced a completely wrong
conclusion on this project.

If you compute a device's internal rates by sampling its counters and dividing
by **host wall-clock time**, and the device is simultaneously streaming heavy
data, your reader falls behind. The last status message you parse is *older*
than the moment you stop timing, so every rate is scaled down by the same
factor. It looks exactly like the device has slowed down.

The tell: two unrelated counters that should be independent both come out low
by an identical ratio.

**Use the device's own timebase** — an uptime or tick field in its telemetry —
as the denominator, never the host clock. On this project that changed an
apparent "sensor collapsed from 8.8 to 1.9 frames per second" into the truth,
which was 8.7 throughout.

---

## 11. Checklist for the "different camera, same wires" case

In the order most likely to be the answer:

1. **Pin naming convention differs.** The new device's `A`/`B` may be its
   driver where the old device's `A`/`B` was its receiver. **Swap the two
   pairs** and retry. This is free and is the most likely cause.
2. **It is 2-wire half duplex, not 4-wire full duplex.** Count the signal
   terminals. If two plus ground, the crossover model does not apply.
3. **Polarity within a pair is inverted.** Symptom is garbage, not silence.
   Swap the two wires of one pair.
4. **Different baud or framing.** Measure the bit period on a scope rather than
   guessing; check for parity and stop-bit differences too.
5. **The device is request-response and is behaving correctly** by saying
   nothing. Verify before rewiring anything.
6. **Termination now wrong** for the new device — it may already have a
   resistor fitted internally, in which case an external one gives you two.
7. **Driver enable** on the new device may need asserting, or may be under
   software control and default to off.
8. **Different common-mode or isolation situation** — grounds joined?

Work down that list *before* re-examining anything that already worked. The
cable is the least likely thing to have changed.

---

## Appendix: a minimal, dependency-light listener

Useful for any RS-422 device, not just this project. Prints a byte rate and a
hex preview, which is enough to distinguish all three step-2 outcomes.

```python
import sys, time, serial

port = serial.Serial(sys.argv[1], int(sys.argv[2]), timeout=0.2)
try:
    port.set_buffer_size(rx_size=1 << 20)
except Exception:
    pass

seconds = 5.0
end = time.time() + seconds
total, preview = 0, bytearray()
while time.time() < end:
    chunk = port.read(4096)
    total += len(chunk)
    if len(preview) < 64:
        preview.extend(chunk[: 64 - len(preview)])
port.close()

print(f"{total} bytes in {seconds:.0f} s  ({total / seconds:.0f} B/s)")
print("first bytes:", preview.hex(" ") or "(none)")
```

Read it as:

- `0 bytes` → wiring, power, or the device only speaks when spoken to.
- Bytes that repeat a recognisable pattern → link good, decode the protocol.
- Bytes that are mostly `00` or `ff`, or change wildly run to run → baud,
  framing, or polarity.
