# Artisan Configuration Context — airRoaster

## Goal

Configure Artisan to:
1. Read BT and ET from two Phidget temperature modules on a networked HUB5000
2. Send fan and heat control commands to an ESP32 roaster controller over WebSocket

---

## Hardware

| Device | Role |
|--------|------|
| ESP32 (airRoaster firmware) | Roaster controller — controls heat and fan via I2C AC dimmers |
| Phidget HUB5000 | VINT hub, networked at `192.168.1.232` (also reachable as `hub5000.local`) |
| Phidget TMP1200 on HUB VINT port 0 | RTD input, PT1000 probe → **BT (bean temperature)** |
| Phidget TMP1101 on HUB VINT port 1 | 4-channel thermocouple input, K-type probe on channel 0 → **ET (exhaust temperature)** |

---

## ESP32 WebSocket Interface

- URL: `ws://<esp32-ip>:81`
- The ESP32 IP is shown on its OLED display at startup.

### Polling (Artisan → ESP32)

Artisan sends on its sample interval:
```json
{"command": "getData", "id": 12345, "roasterID": 0}
```

ESP32 responds:
```json
{"id": 12345, "data": {"BT": 0.0, "ET": 0.0}}
```
BT and ET are currently stubbed at 0.0 — real values come from Phidgets (see below).

### Control commands (Artisan sliders → ESP32)

Slider actions use Artisan's WebSocket `send()` syntax:

| Slider | Action string | Effect |
|--------|--------------|--------|
| Fan | `send("OT2;{}")` | Sets fan level 0–100% |
| Heat | `send("OT1;{}")` | Sets heat level 0–100% |

The firmware also accepts `send({{"command":"OT2","value":{}}})` JSON envelope format — both work.

### Status broadcast (ESP32 → all clients, unsolicited)

```json
{"pushMessage": "status", "data": {"heat": 60, "heatReq": 60, "fan": 50, "ilCap": 100, "ilSoft": false}}
```

| Field | Description |
|-------|-------------|
| `heat` | Actual heat level sent to dimmer (%) |
| `heatReq` | Requested heat level before interlock (%) |
| `fan` | Fan level (%) |
| `ilCap` | Current heat ceiling from interlock (0=blocked, 100=unrestricted) |
| `ilSoft` | `true` = soft (linear ramp) interlock; `false` = hard (binary) interlock |

---

## Key References

- Artisan WebSocket protocol: https://artisan-scope.org/devices/websockets/
- Artisan Phidgets configuration: https://artisan-roasterscope.blogspot.com/2017/12/more-phidgets.html
- HUB5000 + dual single-channel modules setup (home-barista forum, post #7 by Rob W):
  > Since each VINT module delivers only one channel but Artisan's main device expects two,
  > use an Extra Device for the second module and remap its channel to ET via the symbolic
  > formula `Y3` in the Symb ET/BT tab.

---

## Artisan Configuration Steps

### 1. Main device (Config → Device → 1st tab "ET/BT")

- **Meter**: `Phidget TMP1200`
  - This provides BT from VINT port 0, PT1000.
  - The ET slot will be filled via symbolic remap from the extra device (see step 4).

### 2. Phidgets tab (Config → Device → 4th tab "Phidgets")

- **TMP1200**: VINT port `0`, channel `0`, RTD type `PT1000`
- **Network**: ✓ ticked, host `192.168.1.232`, port `5661`

### 3. Extra Devices tab (Config → Device → Extra Devices)

Add **one extra device**:
- Type: `Phidget TMP1101 4xTC`
- VINT port `1`, channel `0`, thermocouple type `K`
- Untick **LCD and curve for channel 2** (TMP1101 returns only one signal here)
- Untick **LCD and curve for channel 1** (data will be remapped to ET — no need to display it separately)

Add **one more extra device** (for WebSocket slider control):
- Type: `WebSocket` pointed at `ws://<esp32-ip>:81`
- Untick LCD and curve for both channels (BT/ET from this device are stubbed at 0.0 and ignored)
- Sliders send via this connection

### 4. Symb ET/BT tab (Config → Device → 2nd tab)

- **ET Y(x)**: `Y3`

`Y3` is the symbolic variable for channel 1 of the first extra device (the TMP1101), routing its reading into the main device's ET slot.

### 5. Sliders (Config → Events → Sliders)

| Slider | Action type | Command |
|--------|-------------|---------|
| Slider 0 (Fan) | WebSocket | `send("OT2;{}")` |
| Slider 1 (Heat) | WebSocket | `send("OT1;{}")` |

Both sliders send over the WebSocket connection to the ESP32.

---

## Firmware Fan Interlock (informational)

The ESP32 firmware enforces a fan interlock — heat is suppressed when fan is too low.
Two modes, toggled by sending `IL` over WebSocket or serial:

| Mode | Behaviour |
|------|-----------|
| Hard (default) | Heat = 0 if fan < 23%, else unrestricted |
| Soft | Heat linearly capped: 30% at fan=23%, 100% at fan≥30% |

The `ilCap` field in the status broadcast shows the current ceiling.

---

## Existing .aset file

A partially-configured Artisan settings file exists at `artisan/airRoaster.aset` in the firmware repo. It was originally configured with:
- WebSocket as main device (now should move to extra device / control only)
- Sliders using Serial action (now updated to WebSocket `send()`)
- Phidgets not yet configured

The .aset file needs to be updated to reflect the configuration described above.
