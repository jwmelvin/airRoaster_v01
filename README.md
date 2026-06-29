# airRoaster v01

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
| `IL` | `IL` | Toggle interlock mode between hard and soft (see [Fan interlock](#fan-interlock)) |
| `LOG` | `LOG` | Retrieve the error log (sent only to requesting client) |

All unsolicited messages use Artisan's push message envelope so any client can use a consistent format:

**Status broadcast** (sent to all clients on any state change, and to new clients on connect):

```json
{"pushMessage": "status", "data": {"heat": 60, "heatReq": 60, "fan": 50, "ilCap": 100, "ilSoft": false}}
```

| `data` field | Description |
|-------|-------------|
| `heat` | Actual heat level applied to the dimmer (%) |
| `heatReq` | Requested heat level before interlock (%) |
| `fan` | Fan level (%) |
| `ilCap` | Current heat ceiling imposed by the interlock (0–100%); `0` means heat is fully blocked, `100` means unrestricted |
| `ilSoft` | `true` if soft (linear) interlock mode is active; `false` for hard (binary) mode |

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

## OLED display layout

```
+--------------------------------+
|  airRoaster vX.Y.Z             |  ROW1 (firmware version)
|  Heat: 60        Req: 60       |  ROW2
|  Fan: 50                       |  ROW3
|  IL:H ok                       |  ROW4 (interlock status — see below)
|                                |  ROW5
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

A ready-to-import Artisan settings file is provided at `artisan/airRoaster_v01.aset` in this repo.

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
