# airRoaster v01

ESP32-based controller for a hot-air coffee roaster. Controls a heating element and fan independently via two [RBDimmer DimmerLink](https://www.rbdimmer.com/docs/dimmerlink-I2CCommunication) I2C AC dimmers, with a WebSocket interface for remote control and an OLED display for local status.

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | ESP32 (any variant with I2C and WiFi) |
| Display | SH1107 64×128 OLED, I2C address `0x3C` |
| Heat dimmer | RBDimmer DimmerLink, I2C address `0x51` |
| Fan dimmer | RBDimmer DimmerLink, I2C address `0x52` |

Both dimmers are on the same I2C bus as the display. The dimmer curve is set to mode `1` at startup.

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
| `FAN_INTERLOCK_THRESHOLD` | `30` | Minimum fan level required to allow heat (%) |
| `WS_PORT` | `81` | WebSocket server port |

---

## Startup sequence

1. Display and I2C bus initialize
2. ESP32 connects to WiFi (blocks until connected)
3. WebSocket server starts on port `81`
4. Both DimmerLink devices initialize — retried up to 3 times each; failures are logged
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
| `LOG` | `LOG` | Retrieve the error log (sent only to requesting client) |

**Status broadcast** (sent to all clients on any state change, and to new clients on connect):

```json
{"heat": 60, "heatReq": 60, "fan": 50, "interlock": false}
```

| Field | Description |
|-------|-------------|
| `heat` | Actual heat level applied to the dimmer (%) |
| `heatReq` | Requested heat level before interlock (%) |
| `fan` | Fan level (%) |
| `interlock` | `true` if fan is below threshold and heat is suppressed |

**Error broadcast** (sent to all clients when an error is logged):

```json
{"error": "DimmerLink 0x51 ERR_PARAM (0xFE)"}
```

**Log response** (sent only to the client that sent `LOG`):

```json
{"log": "DimmerLink 0x52 not ready after 3 tries"}
{"log": "no errors"}
```

### Serial (115200 baud)

Accepts the same command set as WebSocket, terminated by newline. Maximum command length is 32 bytes. `LOG` prints the error log to Serial, one entry per line prefixed with `[LOG]`.

---

## Fan interlock

Heat is suppressed whenever `fanLevel < FAN_INTERLOCK_THRESHOLD` (default 30%). This prevents the heating element from running without adequate airflow.

- `heatReq` tracks what the user commanded
- `heat` reflects what is actually applied — `0` while interlocked
- Heat resumes automatically to `heatReq` as soon as the fan rises above threshold

The interlock is re-evaluated every loop iteration, so it responds immediately regardless of how the fan level changes.

---

## OLED display layout

```
+--------------------------------+
|  (reserved)                    |  ROW1
|  Heat: 60        Req: 60       |  ROW2
|  Fan: 50                       |  ROW3
|  INTERLOCK                     |  ROW4 (shown only when interlocked)
|                                |  ROW5
|  192.168.1.42                  |  ROW6 (IP address or "No WiFi")
|                                |  ROW7
|                                |  ROW8
+--------------------------------+
```

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
| BT channel | `BT` node |
| ET channel | `ET` (inlet temp, stubbed until sensor added) |
| Slider 0 | Fan — sends `OT2;<value>` |
| Slider 1 | Heat — sends `OT1;<value>` |

Temperature channels (`BT`, `ET`) return `0.0` until physical sensors are wired and the `btTemp`/`etTemp` globals are populated in the firmware.

---

## Dependencies

Install via Arduino Library Manager:

- [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit SH110X](https://github.com/adafruit/Adafruit_SH110x)
- [WebSockets by Markus Sattler](https://github.com/Links2004/arduinoWebSockets)

---

## License

MIT — see [LICENSE](LICENSE).
