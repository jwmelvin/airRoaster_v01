# Development plan

## inlet-temp control

### framing

The airRoaster now measures inlet temperature, which is a primary process variable. The current system relies on external commands to regulate the heater to maintain a desired inlet temperature. That suffers from the slower reporting cycle by which Artisan receives temperature updates and sends heater commands (1 Hz). Improvement (tighter control) can come from internalizing the control loop. It will also allow flexible control strategies that Artisan may not.

### objectives

Develop a switchable operating mode that accepts "inlet" commands to set the desired inlet temperature, and implements feedback control from the measured inlet temp to modulate the heater with the tightest possible control. The control strategy and tuning should prioritize robustness against flow variation (e.g., from external fan-speed changes). Generally, the inlet setpoint will change gradually during a roast, with large transitions happening only at the start of a roast. 

### design considerations

- as an initial gate, consider whether this change (or other project attributes) would favor moving from the Arduino environment to ESP-IDF (https://docs.espressif.com/projects/esp-idf/en/v6.0.1/esp32s3/index.html)

- one reason for that potential migration is that the microcontroller for this project, ESP32-S3, has dual cores and the temperature-measurement-and-control aspects might benefit from separating them from the interface (WiFi, websocket, and OLED) aspects. I do not know if Arduino supports dual-core implementations but ESP-IDF does.


### constraints

- Operate the control loop with deterministic timing.
- Branch the project so that work can progress on this new capability without disturbing the current code.
- 


### references

- PID library for Arduino environment: https://github.com/br3ttb/Arduino-PID-Library
-- author's explanations: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
-- explanation of the library's POM mode:  http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/
-- and: http://brettbeauregard.com/blog/2017/06/proportional-on-measurement-the-code/ 

---

## plan (2026-06-29)

Branch: `feature/inlet-control`. Planning only at this stage — no firmware
written yet.

### Gate decision 1 — stay in Arduino, do NOT migrate to ESP-IDF (yet)

The two reasons floated for ESP-IDF (deterministic timing, dual-core
separation) are both achievable *inside the Arduino-ESP32 core*, which is
itself a thin layer over ESP-IDF + FreeRTOS:

- **Dual core is already available.** The Arduino `loop()` runs as a FreeRTOS
  task (`loopTask`) pinned to core 1; core 0 runs the WiFi/BT stack. We can
  create our own control task with `xTaskCreatePinnedToCore()` — no migration
  required.
- **Deterministic cadence is already available.** A dedicated FreeRTOS task
  using `vTaskDelayUntil()` (fixed wake period, not `delay()`) at a raised
  priority gives a jitter-bounded loop independent of WebSocket/OLED work.

Against migration: a port would have to replace the whole interface stack that
already works — `WebSocketsServer` (Links2004), `Adafruit_SH110X`/`GFX`,
`Adafruit_MAX31865`/`MAX31855`, `WiFi`. That is weeks of yak-shaving for no
control benefit the Arduino core can't deliver. **Recommendation: keep Arduino;
revisit ESP-IDF only if we later hit a concrete wall (e.g. needing TLS, OTA at
scale, or sub-millisecond ISR-level timing the Arduino core obstructs).**

### Gate decision 2 — control library

Use **Dlloydev/QuickPID** for the controller and **Dlloydev/sTune** for
characterization/autotune (same author, designed to interoperate, both
ESP32-clean, actively maintained — unlike br3ttb's original which is feature-
frozen). QuickPID adds what we need over the original: selectable
Proportional-on-Error / Proportional-on-Measurement *blend* (per Beauregard's
POM articles), multiple anti-windup modes, and a TIMER mode that fits a
fixed-cadence FreeRTOS task. sTune's open-loop **s-curve inflection** test
identifies a first-order-plus-dead-time (FOPDT) model in ~½τ — ideal for a
thermal plant and the basis for an operator-runnable tuning routine.

References vetted: br3ttb library still works but is the wrong long-term choice;
QuickPID + sTune supersede it. (Kilill's ESP-IDF fork is only relevant if we
ever migrate.)

### Actuator note (simplifies everything)

The heater is a phase-cut AC dimmer that already accepts a **continuous 0–100
level** over I²C (`setLevel(HEAT_ADDR, …)`). So the PID output maps *directly*
to heat level — no time-proportioning relay/PWM layer needed. The control
output path is just: `pidOutput → requestedHeatLevel → applyInterlock() →
setLevel()`.

### Control strategy — robustness against flow variation is the design driver

Airflow (fan level, possibly changed externally mid-roast) changes BOTH the
plant gain and the time constant of inlet temperature vs. heat: more flow ⇒
lower steady inlet temp for the same power AND a faster, shorter-τ response.
A single fixed PID tuned at one flow will be sluggish or unstable at another.
Three layers, in priority order:

1. **Feedforward from a static power map** `heat_ff = f(setpoint, fan)`.
   Steady-state heater power needed scales ~linearly with mass-flow ×
   (T_set − T_ambient). A small calibrated map gives most of the command
   instantly when fan changes; the PID only trims the residual. This is the
   single biggest robustness win and largely neutralizes flow disturbances
   before the feedback loop has to react.
2. **Gain scheduling on fan level — conditional.** Because fan level is a
   *known* input, gains *can* be scheduled across fan breakpoints and
   interpolated. But the actual operating band is narrow (50–65, mostly 55–60;
   see Operator decisions), so a single tune at the center is the starting
   point and scheduling is added only if band-edge validation needs it.
3. **Robust feedback tuning (IMC / lambda).** Tune via Internal-Model-Control
   (lambda) rules rather than Ziegler-Nichols: λ is an explicit
   robustness/speed knob (λ ≈ θ for brisk, 2–3θ for conservative). Pick λ on
   the conservative side and validate gain margin at the worst-case (lowest-
   flow, highest-gain) point. Use **Proportional-on-Measurement-leaning blend**
   so the occasional large setpoint step (roast start) doesn't cause overshoot,
   while gradual setpoint moves during the roast stay tight.

Supporting details:
- **Anti-windup** is mandatory: heat saturates at 0/100 *and* the fan interlock
  can cap it below the PID's request. The integrator must be clamped/back-
  calculated against the **actually-applied** heat (post-interlock), not the
  requested value, or it winds up whenever the interlock bites.
- **Derivative** on the inlet thermocouple: derivative-on-measurement + light
  low-pass (the TC is fast but noisy near the dimmers — see hardware/emi.md).
  Start PI-only; add D only if sTune shows it helps.
- **Bumpless transfer**: on engaging inlet mode, seed the integrator with the
  current heat level so there's no step; on disengaging, leave heat as-is.

### Mode + command design

New switchable closed-loop mode layered on the existing manual path:
- `INLET <degC>` — set inlet setpoint AND engage closed-loop mode (bumpless).
- `INLET OFF` — disengage, hold last heat (manual).
- Sending `OT1 <value>` (manual heat) auto-disengages inlet mode — gives the
  operator an instant manual override with no extra command.
- `TUNE` (operator routine) — run an sTune open-loop step at the *current* fan
  level, report identified K/τ/θ + suggested gains over WebSocket/serial, and
  optionally store them into the gain-schedule slot for that fan band. Operator
  repeats at low/med/high fan to populate the schedule.
- Status broadcast gains: `mode` (manual|inlet), `inSV` (setpoint), and the
  inlet temp is already published as `IN`.

Artisan drives the setpoint via a background profile (see the utility below,
which generates an `.alog` whose background curve becomes the inlet SV) or a
dedicated slider/event mapped to the `INLET` command — mirroring how OT1/OT2
sliders work today.

### Concurrency / timing architecture

- **Core 1 (interface task = Arduino `loop()`):** WebSocket, OLED, command
  parsing, WiFi servicing — unchanged, non-deterministic is fine here.
- **Core 0 (new `controlTask`, pinned, raised priority, `vTaskDelayUntil`):**
  read inlet TC → run QuickPID (TIMER mode) → apply feedforward + gain schedule
  → write heat through the interlock. Fixed cadence ~5–10 Hz (the TC + thermal
  plant don't justify faster; deterministic period matters more than rate).
- **Shared state** (setpoint, mode, fan level, heat level, gains) guarded by a
  small mutex or made atomic. Keep the critical section tiny.
- The other sensors (BT/ET RTDs) and their robust-read logic can stay on the
  interface cadence or move into the control task; decide during implementation.

### Architecture decision (resolved 2026-06-29)

After working through the FreeRTOS dual-core option: **cooperative single-core**
for v1, with the control law isolated in a `controlStep()` *seam* (operates only
on shared globals) so it can later lift into a pinned task unchanged. Rationale:
the ESP32-S3 has two I2C controllers and a spare SPI, so a lock-free dual-core
partition *is* possible (heat dimmer on `Wire1`/control core; OLED+fan+all
sensors/SPI on the interface core; cross-core state as 32-bit-atomic scalars) —
but it needs a hardware rewire (heat dimmer → `Wire1`) in the EMI-sensitive zone
and the plant's tens-of-seconds time constants don't require sub-10 ms
determinism. `STAT` reports a control-cadence jitter watch as the evidence that
would justify migrating later.

Also decided to implement a **compact inline PID** rather than QuickPID for the
loop itself — it gives exact control over back-calculation anti-windup against
the *post-interlock* applied heat, with no dependency to install/compile here.
sTune is still the intended tool for the `TUNE` step-test routine (phase 3).

### Phased work

1. ~~Scaffolding: `controlStep()` seam, fixed-cadence call from `loop()`,
   jitter watch.~~ **done (v0.3.0)** — cooperative, not a task.
2. ~~Closed-loop PI(D) with back-calculation anti-windup against post-interlock
   heat; `INLET`/`INLET OFF`/`OT1`-override mode switching with bumpless
   transfer; `PID` live-tuning + `STAT` commands.~~ **done (v0.3.0)** — gains
   are untuned placeholders.
3. ~~`TUNE` routine (open-loop step test) + reporting; FOPDT fit + SIMC PI
   suggestions.~~ **done (v0.4.0)** — implemented inline (two-point FOPDT fit,
   SIMC tight/conservative gains) rather than with sTune, for full control over
   step safety/abort and to stay dependency-free + compile-checkable here.
   Operator still needs to run it at center fan (~57) and spot-check 50 / 65.
4. Feedforward power map (2–3 points across 50/57/65) + lambda/IMC gains from
   step (3); add gain scheduling only if band-edge validation needs it. **next**
5. Validation roasts: step + ramp setpoints, deliberate mid-roast fan changes;
   measure tracking/overshoot/settling and confirm robustness. Tune λ.
6. ~~Docs: README control-mode section.~~ **done (v0.3.0)** — Artisan
   integration + `.aset` for driving the setpoint still to do in phase 4.

### Operator decisions (resolved 2026-06-29)

1. **Overshoot tolerance is generous; tracking is the priority.** ~5 s at
   +10–20 °F (≈ +5.5–11 °C) overshoot at roast start is acceptable. What
   matters most is tight *tracking* during the roast and good rejection of
   airflow changes. → Tune on the **brisk** side (smaller λ, ≈ θ–1.5θ);
   prioritize setpoint tracking + flow-disturbance rejection over startup-step
   overshoot suppression. The generous startup tolerance means we do **not**
   need a heavy Proportional-on-Measurement bias to tame the opening step — a
   light POM blend or even proportional-on-error is fine. Validate on the
   tracking/disturbance metrics, not the startup step.
2. **Fan operates in a narrow band: 50–65, usually 55–60.** This is small
   enough that the plant gain/τ won't swing much across it, so **full gain
   scheduling is likely unnecessary.** Revised approach:
   - Identify the plant once with sTune at the **center (~57)**, and tune a
     single robust PID there.
   - Keep the **feedforward power map** anyway — it's what gives fast rejection
     of mid-roast airflow changes (the stated priority), and over a 50–65 span
     it's nearly linear so a 2–3 point map (50 / 57 / 65) suffices.
   - Add gain scheduling **only if** step (5) validation shows the single tune
     degrades at the band edges. Spot-check sTune at 50 and 65 to confirm the
     model barely moves before deciding.
3. **`OT1` is an instant manual override** — confirmed. Any `OT1 <value>`
   immediately disengages inlet mode and applies the manual heat level.

# utilities

## Artisan profile generator so a background can drive SV through Inlet control parameter

python3 make_inlet_background.py --out bkgnd_001-base_330-550_5.30.alog --mode ror_endpoint --no-mirror --template bkgnd_001-base.alog \
--t_drop 330 \
--T_start 350 --T_drop 500 \
--ror_start 50

