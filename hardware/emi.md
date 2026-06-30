# EMI / Sensor Noise — diagnosis and mitigation

How the AC dimmers couple noise into the temperature sensors, what's been done
about it, and what's left. Referenced from the sensor code in
[`../airRoaster.ino`](../airRoaster.ino) (`serviceRtd()` and the
robust-read constants).

---

## TL;DR — priority actions

| # | Action | Side | Status |
|---|--------|------|--------|
| 1 | Firmware tolerance: glitch-reject, debounce, hold-last-good | software | **done** (v0.2.0) |
| 2 | VIN–GND decoupling at each MAX31865 board (10 nF + 100 nF) | sensor | **done** |
| 3 | RC filter (100 Ω + 10 nF) on each RTD lead | sensor | **done** (helps lead pickup, not the reference path) |
| 4 | Twist each dimmer→load pair (line + return together) | source | **done** |
| 5 | Foil + copper-mesh shield on load runs, single-end grounded at enclosure | source | **done** |
| 6 | Dedicated earth bond to roaster body / enclosure (0.2 Ω) | grounding | **done** (was ~2 Ω) |
| 7 | Relocate 12→5 V converter below the divider | supply | **done** |
| 8 | Caps on R-78W output + input, and 5 V point-of-load (see below) | supply | **in progress** |
| 9 | SPI: shorten runs + 33 Ω series resistors at the Feather | sensor | **todo** |
| 10 | Clamp-on / toroid ferrite at the dimmer end of each load cable | source | **optional** |

The source-side and grounding work (4–7) is done. Remaining: finish the supply
decoupling (8) and tidy the SPI wiring (9), which is now the weakest link.

---

## Symptom

The connected PT1000 RTD (MAX31865) periodically stops reporting new values
during a roast and, before the firmware fix, would drop to 0 °C (= 32 °F in
Artisan). The K-type thermocouple (MAX31855) on the same SPI bus is unaffected.

Representative fault log (from `LOG` over WebSocket — see
[`../artisan/dashboard.html`](../artisan/dashboard.html)):

```
MAX31865 ET fault: 0x60   <- most common
MAX31865 ET fault: 0x64
MAX31865 BT fault: 0x08
```

A roast log (`airRoaster_14.alog`) showed the working RTD dropping out for
9–21 s at a time, several times between 3 and 7 minutes in.

---

## Fault-code decoding (MAX31865 fault register, 0x07)

| Bit | Mask | Meaning |
|-----|------|---------|
| D7 | 0x80 | RTD resistance above High Fault Threshold |
| D6 | 0x40 | RTD resistance below Low Fault Threshold |
| D5 | 0x20 | REFIN− > 0.85 × VBIAS |
| D4 | 0x10 | REFIN− < 0.85 × VBIAS (FORCE− open) |
| D3 | 0x08 | RTDIN− < 0.85 × VBIAS (RTD− / FORCE− open) |
| D2 | 0x04 | Overvoltage / undervoltage on an analog input |

Observed:

- **`0x60` = D6 + D5** — low RTD reading **+ reference low-side (REFIN−) pulled high.**
- **`0x64` = D6 + D5 + D2** — same, **plus an input transient outside the valid window.**
- **`0x08` = D3** — RTD− effectively open. This came from the *unconnected*
  board (no probe); it is expected and is gated off in firmware. The single
  connected probe is the **BT** (bean) probe; the no-probe board is **ET** and is
  disabled via `RTD_ET_ENABLED`.

The **D2 bit in `0x64` is the tell**: it fires only when an input briefly swings
outside its valid range — the fingerprint of an *injected transient*, not a
steady wiring fault. And **D5 places the disturbance in the reference path**
(the 4300 Ω reference resistor / VBIAS network), which is *internal to the
board* — not on the probe leads.

---

## Diagnosis

**Key clue: the thermocouple never faults, but the RTD does.** If this were
generic SPI-bus corruption or a dirty 3.3 V/5 V rail, both SPI devices would
suffer. They don't. That rules out "the whole bus is noisy" and points at what
makes the MAX31865 different:

1. It makes a **ratiometric** measurement against VBIAS through a tiny bias
   current. Disturb VBIAS or the reference divider and the reading moves.
2. It runs **analog fault comparators** (REFIN−, RTDIN−, over/under-voltage)
   continuously. A fast transient can trip a comparator mid-conversion even when
   the underlying ADC value would have been fine.

The phase-cutting dimmers (TRIAC fires partway through each mains half-cycle)
emit sharp broadband transients twice per cycle. These couple into wiring by
both magnetic (di/dt through load-wire loop area) and electric (dv/dt) fields.
The MAX31865's reference circuit is the susceptible victim.

**Not correlated with heat level** — confirmed in testing. That fits, because
the **fan dimmer runs continuously** through a roast (same RBDimmer module),
so its transients are present regardless of the heater setting. Both dimmers
are equal suspects; don't fixate on the heater.

### Why the lead RC filters didn't fully fix it

The 100 Ω + 10 nF filters on the RTD **leads** address pickup on the RTD wires
(the RTDIN path). But the dominant `0x60`/`0x64` faults are in the **REFIN /
VBIAS** path, which is on the board and not reachable from the leads. So the
lead RC is correct and worth keeping (it protects against RTDIN-side faults and
helps the open-lead case), but it cannot suppress the reference-path noise — which
is why dropouts persisted after adding it.

### Note on the 50/60 Hz notch filter

The MAX31865 has a selectable 50/60 Hz notch (Adafruit library default 60 Hz,
correct for US mains). It rejects *mains-frequency* pickup but **not** the
broadband dimmer-edge transients, so it is not the fix here — but it is set
correctly and should stay at 60 Hz.

---

## What's been done

### Software (v0.2.0)

`serviceRtd()` in [`../airRoaster.ino`](../airRoaster.ino) makes the
read robust instead of trusting a single fault byte:

- **Same-cycle glitch rejection** — two reads per cycle; if they disagree by
  more than `RTD_READ_DISAGREE_C` (2 °C) the SPI frame was likely corrupted, so
  the value is held and retried next cycle.
- **Plausibility window** — a reading is accepted only if it falls between
  `RTD_VALID_MIN_C` (−20 °C) and `RTD_VALID_MAX_C` (600 °C); a transient fault
  flag on an otherwise in-range value is ignored.
- **Debounce** — a channel is only declared faulted after `SENSOR_FAULT_DEBOUNCE`
  (4) consecutive bad reads, so brief transients don't blank the curve.
- **Hold-last-good** — during a fault the global keeps its last valid value
  rather than dropping to 0 °C, so Artisan sees a momentary hold (much better for
  the curve and RoR than a spike to 32 °F).
- **Converter recovery** — `clearFault()` is followed by re-enabling bias and
  auto-convert (clearing the fault resets config bits, which would otherwise stop
  conversions).
- **Rate-limited logging** — repeated faults are logged at most every
  `SENSOR_REFAULT_LOG_MS` (5 s) so the 8-entry error log isn't flooded.

This makes the channel ride through brief EMI events regardless of source, but it
is *tolerance*, not a cure — the hardware items below reduce the events
themselves.

### Hardware (sensor side)

- **VIN–GND decoupling** at each MAX31865 board input — 10 nF + 100 nF,
  stiffening the supply feeding VBIAS.
- **RC filter** (100 Ω series + 10 nF to GND) on each RTD lead — two per board,
  on RTD+ and RTD−. RC ≈ 1 µs, well inside the MAX31865's tolerance.

### Hardware (source + grounding + supply)

- **Twisted load pairs** — each dimmer→load line+return twisted together to
  collapse loop area (magnetic emission).
- **Foil + copper-mesh shield** over the load runs, **single-end grounded at the
  enclosure** (stops short of the grounded roaster body to avoid a ground loop;
  see §3 under *Source suppression* for the foil+mesh and single-end rationale).
- **Dedicated earth bond** added — roaster body / enclosure to incoming earth is
  now **0.2 Ω** (was ~2 Ω). Improves both fault safety and the shield's ground
  reference.
- **12→5 V converter (R-78W) relocated below the divider**, moving the switching
  node to the mains side; only the filtered 5 V now crosses to the sensors.

### Considered but not feasible

- **Cap directly across the reference resistor (REFIN+ / REFIN−)** would target
  the `0x60` path directly, but the SMD pads are too small/precise to add
  reliably by hand. Abandoned in favour of supply decoupling + source
  suppression.

---

## Capacitor summary

Consolidated list of every decoupling/filter cap in the system. The first three
rows are installed; the supply-rail rows are the remaining additions.

| Location | Caps | Purpose | Status |
|----------|------|---------|--------|
| Each RTD lead → GND | 10 nF (+ 100 Ω series) | RTD-lead pickup filter (RTDIN path) | installed |
| Each MAX31865 VIN/3V3 → GND | 10 nF + 100 nF | board supply decoupling at VBIAS | installed |
| **R-78W 5 V output** | **100 µF electrolytic (low-ESR) + 100 nF ceramic** | absorb switching ripple at the source | add |
| **R-78W 12 V input** | **10 µF** | switcher input decoupling + stops switching current conducting back up the 12 V line | add |
| **5 V arrival, top side near Feather** | **47–100 µF bulk** | point-of-load; catches noise picked up crossing the divider | add |
| 3.3 V distribution node (optional) | 10 µF bulk | rounds out the 3.3 V rail (likely already covered by the Feather LDO) | optional |

Notes:
- Place the R-78W caps **as close to the converter pins as the terminal block
  allows** — lead length undoes the benefit.
- The 5 V point-of-load cap matters specifically because the converter now lives
  below the divider, so the 5 V output travels across to the sensors: filter hard
  at the source **and** again where it arrives.

## SPI wiring

The remaining weak link. The SPI bundle from the three breakouts to the Feather
is long and the signals aren't paired with grounds. **Caps are not the right tool
here** — capacitance on SPI lines rounds the edges and can hurt signal integrity
at the clock rate. Instead, in priority order:

1. **Shorten the runs** (SCK shortened already) and **twist each signal with a
   ground return**, or run the group flat against the grounded panel.
2. **33 Ω series resistors** on SCK / MOSI / MISO **at the Feather end** — damps
   ringing; the closest thing to a component-level fix.
3. The firmware glitch-rejection (v0.2.0) already tolerates the occasional
   corrupted frame, so the SPI wiring doesn't have to be perfect.

---

## Source suppression — reference

> **Status:** items 1, 3, and 6 below are **done** (twisted pairs, foil+mesh
> shield single-end grounded, dedicated 0.2 Ω earth bond). Ferrites (2), the
> blower brush caps (4), and the snubber (5) remain optional. Kept here as the
> technical reference for why each measure works.

The dimmer→fan and dimmer→heater load cables carry the phase-cut current and are
the primary emitters. Attacking the source helps every victim at once.

### 1. Twist each load pair — biggest free win

Radiated magnetic field is proportional to the **loop area** between the
outgoing conductor and its return. Route each load's line and neutral **together
as a tight pair and twist them**; this collapses the loop area and dramatically
cuts magnetic emission. Make sure the return actually runs alongside the switched
conductor rather than taking a separate path.

### 2. Ferrites on the load cables

Add a **clamp-on ferrite or toroid at the dimmer end of each load cable** (and
optionally where the cable exits the enclosure). Pass **both conductors of that
load through the same core** so the 60 Hz fields cancel and only the
high-frequency common-mode component is choked. A few turns through a toroid
beats a single pass-through clamp. Cheap, reversible, no effect on the 60 Hz
power.

### 3. Shield the external runs

Twisting handles the magnetic field; a grounded shield handles the electric
field. Do both.

**Foil + mesh (as built).** This is the standard foil-braid construction:
- **Foil** gives ~100 % coverage and closes the apertures for higher-frequency
  content. Overlap wraps ≥50 %. Watch for foil tape with *non-conductive*
  adhesive — only the exposed face conducts, so the mesh over it ties it together.
- **Copper mesh** provides the low-impedance ground-drain path and mechanical
  robustness foil lacks. **Fairly open mesh is fine** at the frequencies that
  matter here — the holes are tiny vs. wavelength, so E-field attenuation stays
  good. Multiple wraps raise effective coverage and lower shield resistance; two
  layers is plenty.
- Foil and mesh must be in **electrical contact** so they bond as one shield.

**Shield grounding — one end, at the enclosure (source).** The cable is
electrically short at these frequencies, so single-end termination gives
essentially full shielding without a floating-stub penalty, and it avoids a
ground loop between the enclosure earth and the **separately-grounded roaster
body**. Ground at the *source* (enclosure) end so intercepted noise current
returns near the dimmer. **Stop the shield just short of the body** and insulate
the floating end so it can't contact the body (which would sneak-ground the
second end) or a live conductor.

Alternatively, **shielded mains-rated cable** (e.g. SY/CY screened flex) or
**grounded metal conduit** achieves the same thing in one component.

### 4. Blower brush noise (separate source)

The Ametek 116392-00 is a brushed universal motor, so it generates its own
broadband RF from brush arcing, independent of the dimmer. Ferrites on the motor
leads (item 2) help. Motor-suppression X/Y caps across the brushes are the
classic fix but interact with the TRIAC dimmer — add cautiously, ferrites first.

### 5. Snubber across the dimmer output (advanced, optional)

An RC snubber (≈100 Ω + ~0.1 µF X-rated) across the dimmer output softens the
switching edge and reduces broadband emission. **Caveat:** the RBDimmer modules
may already include one; an added snubber passes a little leakage current when
"off" and can affect low-end dimming. Try the wiring measures first.

### 6. Grounding discipline — done

The enclosure/back-panel is **bonded to AC earth**, with a **dedicated bond now
at 0.2 Ω** (was ~2 Ω), and shields/panel return to a single earth point. HF
currents have a defined low-impedance path and don't circulate through signal
ground.

### Supply side — R-78W (RECOM R-78W-0.5)

The 12 V→5 V converter is a switching regulator (~500 kHz). It has been
**relocated below the divider**, so its switching node sits on the mains side and
only the filtered 5 V crosses to the sensors. Decoupling caps (output bulk +
ceramic, input cap, and a 5 V point-of-load cap up top) are listed in the
[Capacitor summary](#capacitor-summary) — those are the remaining additions.

---

## Layout reference

- Low-voltage section is above the white divider; mains below. Dimmers are lower
  left (below the divider). The R-78W now sits **below** the divider, so its
  switching node is on the mains side and only the **filtered 5 V crosses** the
  divider to the sensors.
- Keep sensor cables (especially the RTD leads) flat against the grounded panel,
  short, and crossing any AC wiring at 90° rather than running parallel.
- See [`pins.md`](pins.md) for the full pin/bus map.

---

## Diagnostic workflow

1. Open [`../artisan/dashboard.html`](../artisan/dashboard.html) in a browser, connect to the
   ESP32 IP, and watch the live `error`/`log` stream during a roast.
2. Note which channel (BT/ET) and which fault codes appear, and whether they
   correlate with fan level, heat level, or are constant.
3. Decode codes with the table above. REFIN/over-voltage bits (D5/D2) ⇒ reference
   path / injected transient ⇒ source suppression + supply decoupling. RTDIN/open
   bits (D3/D4) ⇒ check leads, terminals, and the RC filter on that leg.
4. After each mitigation, compare dropout frequency/duration in the Artisan
   `.alog` to gauge improvement.
