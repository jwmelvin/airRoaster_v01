# Pin Usage — ESP32-S3 Feather (Adafruit 5477)

Arduino variant file: `adafruit_feather_esp32s3`

---

## I2C bus

SDA = GPIO 3 (A6), SCL = GPIO 4 (A7)
Pull-ups: 5 kΩ on-board; also exposed on STEMMA QT connector.

| Address | Device | Product |
|---------|--------|---------|
| `0x3C` | SH1107 OLED | Adafruit 4650 FeatherWing |
| `0x51` | DimmerLink — heat | RBDimmer DimmerLink |
| `0x52` | DimmerLink — fan | RBDimmer DimmerLink |
| TBD (`0x76` or `0x77`) | BME688 env sensor | Adafruit 5046 (future) |

The BME688 connects via the STEMMA QT port — no extra wiring needed.

---

## SPI bus

SCK = GPIO 36, MOSI = GPIO 35, MISO = GPIO 37

| GPIO | Feather label | Firmware define | Device | Role |
|------|--------------|-----------------|--------|------|
| 36 | SCK | — | shared | SPI clock |
| 35 | MOSI | — | shared | SPI data out |
| 37 | MISO | — | shared | SPI data in |
| 10 | D10 / A11 | `CS_RTD_BT` | MAX31865 (Adafruit 3648) | Bean temp PT1000 |
| 9 | D9 / A10 | `CS_RTD_ET` | MAX31865 (Adafruit 3648) | Exhaust/ET PT1000 |
| 8 | A5 | `CS_TC_IN` | MAX31855 (Adafruit 269) | Inlet K-type thermocouple |

---

## Reserved / do not use

| GPIO | Label | Reason |
|------|-------|--------|
| 3 | SDA / A6 | I2C in use |
| 4 | SCL / A7 | I2C in use |
| 7 | — | I2C power control (managed by board) |
| 13 | D13 | Built-in red LED |
| 21 | — | NeoPixel power enable |
| 33 | — | NeoPixel data |
| 35 | MOSI | SPI in use |
| 36 | SCK | SPI in use |
| 37 | MISO | SPI in use |

---

## Free GPIOs

| GPIO | Feather label |
|------|--------------|
| 5 | A8 |
| 6 | A9 |
| 11 | A12 |
| 12 | A13 |
| 14 | A4 |
| 15 | A3 |
| 16 | A2 |
| 17 | A1 |
| 18 | A0 |
| 38 | RX (Serial1) |
| 39 | TX (Serial1) |
| 42 | SS (default SPI SS, unused) |
