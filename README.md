# airRoaster

ESP32-based controller for a hot-air coffee roaster. Controls a heating element and fan independently via two [RBDimmer DimmerLink](https://www.rbdimmer.com/docs/dimmerlink-I2CCommunication) I2C AC dimmers, with a WebSocket interface for remote control and an OLED display for local status.

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | ESP32-S3 Feather with 4MB Flash 2MB PSRAM, https://www.adafruit.com/product/5477 (any ESP32 variant with I2C and WiFi) |
| Display | FeatherWing OLED - 128x64 OLED, https://www.adafruit.com/product/4650 (SH1107 64×128 OLED), I2C address `0x3C` |
| Dimmer interface | RBDimmer DimmerLink, https://www.rbdimmer.com/docs/dimmerlink-overview, I2C address heat: `0x51`; fan: `0x52`, curve is set to mode `1` at startup. |
| Dimmers |Dimmers purchased from RobotDyn Official Store on AliExpress. "Dimmer AC module High Power for 40A 600V High Load, 1 Channel, 3.3V/5V logic"; "Dimmer AC module for 16A/24A 600V High Load, 1 Channel, 3.3V/5V logic with current load control" |
| Heater | from sdm2020_tools on eBay and listed as "1 set 230V 3600W 132.387 Heating Element & mica casing for hot air blower guns". Its power in my use is a little higher because I use 240V.|
| Blower | Ametek 116392-00 |

---

## Configuration

### WiFi credentials

Copy the credentials template and fill in your network details. This file is gitignored and should never be committed.

```
cp secrets.h.example secrets.h   # edit WIFI_SSID and WIFI_PASS
```

If `secrets.h.example` does not exist yet, create `secrets.h` manually:

```cpp
#pragma once
#define WIFI_SSID   "your_ssid"
#define WIFI_PASS   "your_password"
```

### Compile-time constants

| Constant | Default | Description |
|----------|---------|-------------|
| `DUTY_STEP` | `5` | Increment/decrement step for UP/DOWN commands (%) |
| `WS_PORT` | `81` | WebSocket server port |
| `DL_INIT_RETRY_DELAY_MS` | `500` | Delay between DimmerLink ready-check retries at startup (ms) |
| `IL_FAN_MIN` | `23` | Fan level below which heat is always 0 (both interlock modes) |
| `IL_FAN_FULL` | `30` | Fan level at or above which heat is fully unrestricted (soft mode) |
| `IL_HEAT_AT_MIN` | `30` | Heat cap (%) when fan is exactly at `IL_FAN_MIN` (soft mode) |

---

## Startup sequence

1. Display and I2C bus initialize
2. Both DimmerLink devices initialize — retried up to 3 times each (`DL_INIT_RETRY_DELAY_MS` apart); failures are logged. Dimmer init happens before WiFi to avoid missing the ready window at power-on.
3. ESP32 connects to WiFi (blocks until connected)
4. WebSocket server starts on port `81`
5. IP address is shown on the display

---

## Interfaces

### WebSocket (`ws://<device-ip>:81`)

The controller speaks two protocols on the same connection:

#### Artisan protocol (request/response)

Artisan polls the device on its sample interval. The controller responds with the current temperature readings.

**Request** (sent by Artisan):
```json
{"command": "getData", "id": 12345, "machine": 0}
```

**Response** (sent by controller):
```json
{"id": 12345, "data": {"BT": 195.3, "ET": 210.0}}
```

`BT` and `ET` are currently stubbed at `0.0` pending sensor integration. When sensors are added, populate the `btTemp` and `etTemp` globals in the firmware.

#### Plain-text commands

Artisan sliders and any other client send plain-text commands. Token delimiters are space, comma, semicolon, or equals sign. Commands are case-insensitive.

| Command | Example | Description |
|---------|---------|-------------|
| `OT1 <value>` | `OT1 60` | Set heat level (0–100%) |
| `OT1 UP` | `OT1 UP` | Increase heat by `DUTY_STEP` |
| `OT1 DOWN` | `OT1 DOWN` | Decrease heat by `DUTY_STEP` |
| `OT2 <value>` | `OT2 50` | Set fan level (0–100%) |
| `OT2 UP` | `OT2 UP` | Increase fan by `DUTY_STEP` |
| `OT2 DOWN` | `OT2 DOWN` | Decrease fan by `DUTY_STEP` |
| `INLET <degC>` | `INLET 165` | Set inlet setpoint and engage closed-loop control (see [Inlet temperature control](#inlet-temperature-control)) |
| `INLET OFF` | `INLET OFF` | Disengage closed-loop control; heat holds at its current level |
| `PID` | `PID` | Report current PID gains, setpoint, and mode |
| `PID <kp ki kd>` | `PID 1.5 0.05 0` | Set PID gains live (for manual tuning) |
| `TUNE` | `TUNE` | Run an open-loop step test and suggest PI gains (see [Tuning](#tuning)) |
| `TUNE <pct>` | `TUNE 20` | Step test with a specific heat step (% points) |
| `TUNE ABORT` | `TUNE ABORT` | Cancel a running step test |
| `TUNE APPLY` | `TUNE APPLY` | Apply the last test's "tight" suggested gains |
| `FF` | `FF` | Report feedforward params and current value (see [Feedforward](#feedforward-airflow-compensation)) |
| `FF <k>` | `FF 0.0045` | Set the feedforward coefficient `ffK` directly |
| `FF AMB <degC>` | `FF AMB 24` | Set the ambient reference temperature |
| `FF CAL` | `FF CAL` | Auto-calibrate `ffK` from the current (steady) operating point |
| `FF OFF` | `FF OFF` | Disable feedforward (`ffK = 0`) |
| `STAT` | `STAT` | Report and reset the control-cadence jitter watch (worst late-fire, µs) |
| `IL` | `IL` | Toggle interlock mode between hard and soft (see [Fan interlock](#fan-interlock)) |
| `LOG` | `LOG` | Retrieve the error log (sent only to requesting client) |

Sending `OT1 <value>` (manual heat) at any time is an **instant override**: it
drops the controller out of closed-loop mode and applies the manual level.

All unsolicited messages use Artisan's push message envelope so any client can use a consistent format:

**Status broadcast** (sent to all clients on any state change, and to new clients on connect):

```json
{"pushMessage": "status", "data": {"heat": 60, "heatReq": 60, "fan": 50, "ilCap": 100, "ilSoft": false, "mode": "manual", "inSV": 0.0}}
```

| `data` field | Description |
|-------|-------------|
| `heat` | Actual heat level applied to the dimmer (%) |
| `heatReq` | Requested heat level before interlock (%) |
| `fan` | Fan level (%) |
| `ilCap` | Current heat ceiling imposed by the interlock (0–100%); `0` means heat is fully blocked, `100` means unrestricted |
| `ilSoft` | `true` if soft (linear) interlock mode is active; `false` for hard (binary) mode |
| `mode` | `"manual"` (heat set directly by `OT1`), `"inlet"` (heat modulated by the closed loop), or `"tune"` (a step test is running) |
| `inSV` | Inlet setpoint (°C) in use when `mode` is `"inlet"` |

**Error broadcast** (sent to all clients when an error is logged):

```json
{"pushMessage": "error", "data": "DimmerLink 0x51 ERR_PARAM (0xFE)"}
```

**Log response** (sent only to the client that sent `LOG`):

```json
{"pushMessage": "log", "data": "DimmerLink 0x52 not ready after 3 tries"}
{"pushMessage": "log", "data": "no errors"}
```

### Serial (115200 baud)

Accepts the same command set as WebSocket, terminated by newline. Maximum command length is 32 bytes. `LOG` prints the error log to Serial, one entry per line prefixed with `[LOG]`.

---

## Fan interlock

The interlock prevents the heating element from running without adequate airflow. It is always active and has two modes, toggled with the `IL` command (default: hard).

### Hard mode (default)

Binary cutoff: heat is forced to `0` when `fan < IL_FAN_MIN`, and fully unrestricted otherwise.

### Soft mode

Linear ramp between two breakpoints:

| Fan level | Heat cap |
|-----------|----------|
| `< IL_FAN_MIN` (23) | 0% (blocked) |
| `IL_FAN_MIN` (23) | `IL_HEAT_AT_MIN` (30%) |
| `IL_FAN_FULL` (30) | 100% (unrestricted) |
| `> IL_FAN_FULL` | 100% |

Between 23 and 30 the cap is interpolated linearly, so heat scales with airflow rather than snapping on at a threshold.

In both modes:
- `heatReq` tracks what the user commanded
- `heat` reflects what is actually applied after capping
- Heat adjusts automatically whenever the fan level changes
- The interlock is re-evaluated every loop iteration

---

## Inlet temperature control

The controller can hold a target **inlet** temperature itself, closing the loop
on-device instead of relying on Artisan's 1 Hz command cycle. This is a
switchable mode layered on top of manual heat control.

### Modes

| Mode | How heat is set |
|------|-----------------|
| `manual` (default) | Heat is whatever `OT1` last commanded |
| `inlet` | Heat is modulated by a PI(D) loop to hold the inlet setpoint |

- `INLET <degC>` engages `inlet` mode (bumpless — no heat step on engage) and
  sets the setpoint. Sending `INLET` again just retargets the setpoint and keeps
  the integrator, so a gradually-changing setpoint during a roast (e.g. driven
  from an Artisan background profile) does not reset the loop.
- `INLET OFF` returns to `manual`, holding heat at its current level.
- `OT1 <value>` is an instant manual override (drops back to `manual`).

### Control law

`controlStep()` runs on a fixed cadence (`CONTROL_PERIOD_MS`, default 250 ms)
from the main loop. The law is intentionally isolated in that one function
operating on shared globals — the *seam* that lets it move to a dedicated
FreeRTOS task later without changing the math (see `work.md`). It is currently
**cooperative** (single-core): adequate for a thermal plant with tens-of-seconds
time constants, and free of the cross-core I²C/SPI bus contention a dedicated
control task would introduce. The `STAT` command reports the worst observed
control-step timing jitter as evidence on whether a dedicated core is ever
warranted.

Key properties:
- **Derivative on measurement** (no setpoint-change kick), low-pass filtered.
- **Back-calculation anti-windup** against the *actually applied* heat — i.e.
  post-interlock — so the fan interlock capping heat does not wind up the
  integrator.
- Starts **PI-only** (`Kd = 0`); the inlet thermocouple is noisy near the
  dimmers (see [hardware/emi.md](hardware/emi.md)), so derivative is added only
  after the plant is characterized.

### Tuning

> ⚠️ The default gains (`pidKp`/`pidKi`/`pidKd` in the firmware) are **untuned
> placeholders** and are not expected to control well. Characterize the plant
> with `TUNE` (below) and apply gains before relying on closed-loop control.

- `PID` reports the current gains, setpoint, and mode.
- `PID <kp> <ki> <kd>` sets the gains live over WebSocket/serial, so you can tune
  without recompiling.

#### Autotune (`TUNE`)

`TUNE` runs an **open-loop step test**: it holds the current heat to measure a
baseline inlet temperature, applies a heat step, records the response, fits a
first-order-plus-dead-time (FOPDT) model (two-point 28.3%/63.2% method), and
suggests PI gains via the SIMC rule at two robustness levels.

**Procedure**

1. Set the fan to a representative level (your normal roasting range) and a
   moderate heat, and let the inlet temperature settle.
2. Send `TUNE` (default step) or `TUNE <pct>` (e.g. `TUNE 20`). The display shows
   `Tuning...` and `mode` becomes `tune`.
3. The test holds baseline (~8 s), steps heat up, and watches until the response
   flattens (or a 180 s cap). Heat returns to baseline and mode returns to
   `manual` automatically.
4. The result is broadcast as a `tune` push message:
   ```json
   {"pushMessage":"tune","data":{"ok":true,"fan":57,"step":15,"dT":42.3,
     "Kp":2.82,"tau":18.5,"theta":3.2,
     "tight":{"kp":1.97,"ki":0.13},"cons":{"kp":0.79,"ki":0.05}}}
   ```
   - `Kp` (°C/%), `tau`, `theta` (s) are the identified plant model.
   - `tight` is the brisker suggestion (tracking-first); `cons` is more
     conservative. Start with `tight`, fall back to `cons` if it overshoots or
     oscillates.
5. Apply gains with `TUNE APPLY` (uses the `tight` set) or `PID <kp> <ki> <kd>`
   to enter a chosen pair manually. Then `INLET <degC>` to run closed-loop.

**Safety / aborts.** The step goes through the fan interlock, so inadequate
airflow aborts the test (`interlock capped step`). It also aborts on over-temp
(`> 280 °C` inlet) and on any `OT1`/`INLET`/`TUNE ABORT`. A failed fit reports
`{"ok":false,"reason":...}`.

Because plant gain and time constant vary with airflow, run `TUNE` at the center
of your fan range (~57). The feedforward below — not the PID gains — is the main
mechanism for robustness against airflow changes.

### Feedforward (airflow compensation)

Steady-state heater power scales with airflow × temperature rise, so the
controller adds a feedforward term:

```
heat_ff = ffK · fan · (SV − ambient)
```

summed into the control output. The PID then only has to trim the residual. The
payoff is **airflow rejection**: when the fan changes mid-roast, `heat_ff` moves
*immediately* in proportion, instead of waiting for the inlet temperature to
drift and the integrator to catch up. One coefficient covers the narrow fan band.

Feedforward is **off by default** (`ffK = 0`). To set it up:

1. Stabilize the roaster in closed loop (or manual) at a representative fan and
   inlet temperature.
2. Send `FF AMB <degC>` with your ambient temperature (once), then `FF CAL`. This
   derives `ffK` from the current operating point:
   `ffK = heat / (fan · (inlet − ambient))`.
3. `FF` reports the result; `FF <k>` sets the coefficient by hand; `FF OFF`
   disables it.

With feedforward calibrated, the integrator settles near zero and the loop
rejects fan-speed changes largely through the feedforward path. Reported as an
`ff` push message: `{"ffK":0.00451,"amb":24.0,"ff":38.7}`.

---

## Commissioning

Bring the controller up in this order. Every step after the flash is a command
sent over WebSocket — the console page at [artisan/dashboard.html](artisan/dashboard.html)
has a **Commissioning sequence** panel that walks these steps with the value
fields prefilled.

1. **Flash.** With the board connected, run `./verify.sh upload` on the host.
2. **Sanity.** Stay in `manual`; confirm sensors, OLED, and Artisan still behave.
   After a few minutes send `STAT` to read control-cadence jitter — this is the
   evidence on whether the cooperative single-core design holds (expect
   single-digit ms).
3. **Characterize.** Set the fan to the center of your range (`OT2 57`) and a
   moderate heat (`OT1 40`), let the inlet temperature settle, then run `TUNE`.
   Eyeball the reported `dT`/`Kp`/`tau`/`theta` for sanity.
4. **Apply gains.** `TUNE APPLY` uses the `tight` suggestion; if it looks twitchy,
   enter the `cons` set by hand with `PID <kp> <ki> <kd>`.
5. **Calibrate feedforward.** Still steady, send `FF AMB <ambient>` then `FF CAL`.
6. **Close the loop.** `INLET <degC>` near the current inlet temp (bumpless), then
   nudge the setpoint and watch tracking.
7. **The real test.** With the loop holding, deliberately change fan speed and
   confirm the inlet temperature barely moves — that is the feedforward earning
   its keep. Tune gains from there.

## OLED display layout

```
+--------------------------------+
|  airRoaster vX.Y.Z             |  ROW1 (firmware version)
|  Heat: 60        Req: 60       |  ROW2
|  Fan: 50                       |  ROW3
|  IL:H ok                       |  ROW4 (interlock status — see below)
|  Inlet SV:165                  |  ROW5 ("Inlet: off" in manual mode)
|  192.168.1.42                  |  ROW6 (IP address or "No WiFi")
|                                |  ROW7
|  B:198.4 E:212.0 I:264.1       |  ROW8 (BT / ET / inlet temps, °C)
+--------------------------------+
```

ROW4 interlock status values:

| Display | Meaning |
|---------|---------|
| `IL:H ok` | Hard mode, fan above threshold, heat unrestricted |
| `IL:S ok` | Soft mode, fan above `IL_FAN_FULL`, heat unrestricted |
| `IL:S cap=N%` | Soft mode, fan in ramp zone, heat capped at N% |
| `IL:H BLOCKED` | Hard mode, fan below `IL_FAN_MIN`, heat forced to 0 |
| `IL:S BLOCKED` | Soft mode, fan below `IL_FAN_MIN`, heat forced to 0 |

---

## Error logging

Errors are stored in a RAM-only circular buffer (8 entries × 64 chars). No flash writes are performed. The log is lost on reboot.

Errors are generated for:
- DimmerLink not ready at startup (after 3 retries)
- DimmerLink error register non-zero after a write (`ERR_SYNTAX`, `ERR_NOT_READY`, `ERR_INDEX`, `ERR_PARAM`, or unknown code)
- DimmerLink error register polled every 5 seconds during operation

Retrieve the log at any time by sending `LOG` over WebSocket.

---

## Artisan integration

A ready-to-import Artisan settings file is provided at `artisan/airRoaster.aset` in this repo.

**Before importing**, open the file in a text editor and replace `<device-ip>` with your ESP32's IP address (shown on the OLED at startup). Then import via **File › Load Settings** in Artisan.

| Artisan setting | Value |
|-----------------|-------|
| Device | WebSocket (id 111) |
| Host | your ESP32's IP |
| Port | 81 |
| Input 1 → BT | `BT` node — bean RTD (the connected probe) |
| Input 2 → ET | `ET` node — second RTD (no probe yet; reads ~0) |
| Input 3 → IN | `IN` node — inlet thermocouple |
| Slider 0 | Fan — sends `OT2;<value>` |
| Slider 1 | Heat — sends `OT1;<value>` |

**Configuring the WebSocket inputs.** In Artisan, under **Config › Port ›
WebSocket**, set the input node names to match the firmware's JSON fields:
**Input 1: `BT`, Input 2: `ET`, Input 3: `IN`** (i.e. `channel_nodes=BT, ET, IN`
in the `.aset`). The node names just select which JSON field feeds each curve.

If a channel reads 0 °C / 32 °F while live data appears on a *different* curve,
the cause is almost always the **firmware** reading the wrong board for that
channel — confirm `CS_RTD_BT` points at the MAX31865 the bean probe is actually
wired to (see [hardware/pins.md](hardware/pins.md)). It is *not* an Artisan
channel-order issue.

The `ET` channel returns `0.0` until a second RTD probe is wired and
`RTD_ET_ENABLED` is set to `1` in the firmware.

### Adding more sensors

Artisan supports up to 10 channels via WebSocket: 2 on the main device plus 2 per extra device (max 4 extra devices), all pointing at the same `ws://<device-ip>:81`. The 10-channel ceiling is a total across the main device and all extra devices combined.

**To add channels** (firmware):
1. Add a global float for the new sensor
2. Extend the `data` object in `handleArtisanRequest()`:
   ```cpp
   snprintf(buf, sizeof(buf),
            "{\"id\":%s,\"data\":{\"BT\":%.1f,\"ET\":%.1f,\"inlet\":%.1f}}",
            idStr, btTemp, etTemp, inletTemp);
   ```

**To expose new channels in Artisan** (up to 10 total):
- For the first 2 channels beyond BT/ET, add node names to `channel_nodes` in the main device's `.aset` entry
- For additional pairs, add WebSocket extra devices in Artisan pointing at the same `ws://<device-ip>:81`, each with its own `channel_nodes` pair — they connect as additional clients and `handleArtisanRequest()` serves them all from the same `data` object with no firmware changes needed

**Beyond 10 channels:** a second WebSocket server on a different port would be required, as Artisan's framework caps at 10 channels per connection group.

---

## Dependencies

Install via Arduino Library Manager:

- [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit SH110X](https://github.com/adafruit/Adafruit_SH110x)
- [WebSockets by Markus Sattler](https://github.com/Links2004/arduinoWebSockets)

---

## License

MIT — see [LICENSE](LICENSE).
