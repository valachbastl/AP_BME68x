# AP_BME68x

Bosch **BME680 / BME688** environmental sensor driver for the ESP-IDF new I2C
master API. Measures temperature, humidity, barometric pressure and gas
resistance.

## Features

- Temperature [°C], humidity [%RH], pressure [hPa] and gas resistance [Ω] in one `read()`
- **BME680 and BME688 auto-detected** (they share chip ID 0x61). The gas
  subsystem differs between them — run_gas bit, gas registers, resistance
  compensation — and the right path is chosen from the `variant_id` register.
  `isBME688()` reports which one was found
- Float compensation from the Bosch BME68x reference implementation
- Configurable oversampling (T/P/H), IIR filter and gas heater (temperature + duration)
- Non-blocking init (`begin()` returns `esp_err_t`, never aborts the app)
- Idempotent device registration; re-calling `begin()` re-verifies the sensor (recovery)
- Sentinel values: `-126.0f` = sensor not found, `-127.0f` = read error

## Installation

### PlatformIO

Add to `platformio.ini`:

```ini
lib_deps =
    https://github.com/valachbastl/AP_BME68x.git
```

Or with a specific version:

```ini
lib_deps =
    https://github.com/valachbastl/AP_BME68x.git#v1.0.0
```

## Usage

### Basic Setup

```cpp
#include "AP_BME68x.h"

// i2c_bus from i2c_new_master_bus()
// Address 0x76 = SDO/GND (Bosch primary), 0x77 = SDO/VCC (Adafruit breakouts).
AP_BME68x bme(i2c_bus, AP_BME68x::ADDR_DEFAULT);

// Register on the bus, verify chip ID, detect variant, load calibration,
// apply default config.
if (bme.begin() != ESP_OK) {
    // sensor not found / bus error — handle gracefully
}

// Optional: change measurement config (defaults applied by begin()).
// Temp 8x, Press 4x, Hum 2x oversampling, IIR 3, heater 320 °C for 150 ms.
bme.configure(8, 4, 2, 3, 320, 150);
```

### Reading Values

```cpp
AP_BME68x_Data d;
if (bme.read(d) == ESP_OK) {
    printf("T: %.2f °C  H: %.1f %%  P: %.1f hPa  Gas: %.0f Ω (valid: %d, heater stable: %d)\n",
           d.temperature, d.humidity, d.pressure, d.gasResistance, d.gasValid, d.heatStable);
}
```

`read()` triggers a forced-mode measurement and blocks for the full conversion
time — T + P + H oversampling cycles plus the gas heater on-time — then a short
confirmation poll. About 185 ms with the default config; more with higher
oversampling or a longer heater. Call it at your task interval — no need to call
`begin()` again.

`gasResistance` carries a compensated reading whenever the gas ADC is valid;
`gasValid` additionally requires the heater to have stabilised (use it as the
"trust this" flag). `heatStable` exposes the heater bit on its own, so a caller
can still publish a best-effort value while the heater settles.

### Error Handling

```cpp
AP_BME68x_Data d;
esp_err_t ret = bme.read(d);

if (ret != ESP_OK) {
    if (d.temperature == AP_BME68x::NO_SENSOR) {
        // Sensor not found on the bus (disconnected or wrong address)
    } else {
        // I2C read error / measurement timeout
    }
}
```

**Always branch on `ret` before scaling or storing the values.** On error every
field holds a sentinel (`-126.0f` / `-127.0f`); pushed through arithmetic — e.g.
converting Ω to kΩ with `/1000` — a sentinel silently turns into a plausible
number (`-0.13`) that downstream systems cannot tell from a real reading.

## API Reference

| Method | Description |
|--------|-------------|
| `AP_BME68x(bus, address)` | Constructor (address 0x76 default), stores params only |
| `begin()` | Register on bus, verify chip ID, detect variant, load calibration, apply default config |
| `read(data)` | Forced-mode measurement → `AP_BME68x_Data` (blocks for the conversion duration, ~185 ms with defaults) |
| `configure(tempOs, pressOs, humOs, iirFilter, gasHeatTemp, gasHeatDurMs)` | Set measurement config |
| `reset()` | Soft-reset the sensor (~10 ms); re-run `begin()` afterwards |
| `isBME688()` | Variant detected by `begin()` — `false` = BME680, `true` = BME688 |
| `measurementDurationMs()` | How long `read()` blocks for the conversion (ms) — size a task period from it |

### `AP_BME68x_Data`

| Field | Unit | Note |
|-------|------|------|
| `temperature` | °C | |
| `humidity` | %RH | |
| `pressure` | hPa | |
| `gasResistance` | Ω | compensated reading when the gas ADC is valid; 0 otherwise |
| `gasValid` | bool | gas ADC valid **and** heater stable — the "trust this reading" flag |
| `heatStable` | bool | `heat_stab_r`: heater reached target temperature this cycle |

### `configure()` parameters

| Parameter | Values | Default |
|-----------|--------|---------|
| `tempOs` / `pressOs` / `humOs` | 0 (skip), 1, 2, 4, 8, 16 | 8 / 4 / 2 |
| `iirFilter` | 0 (off), 1, 3, 7, 15, 31, 63, 127 | 3 |
| `gasHeatTemp` | target °C (0 = gas disabled) | 320 |
| `gasHeatDurMs` | heater duration [ms] | 150 |

## Scope

Single-heater **forced-mode** measurement of temperature, humidity, pressure and
gas resistance, for both the BME680 and the BME688. The BME688 parallel / scan
mode (multiple heater set-points for Bosch BSEC gas classification) is out of
scope — that belongs in a layer above this driver.

## Sentinel Values

| Value | Constant | Meaning |
|-------|----------|---------|
| `-126.0f` | `NO_SENSOR` | Sensor not found on the I2C bus |
| `-127.0f` | `READ_ERROR` | I2C read error / measurement timeout |

## Notes

- BME680 vs BME688: same chip ID (0x61) and identical T/P/H; only the gas path
  differs. `begin()` reads `variant_id` (0xF0) and picks the right registers and
  compensation. LaskaKit / many current breakouts ship the BME688
- A degraded / dead gas element still reports `gasValid` bits but returns a zero
  gas ADC — that reading is rejected (`gasValid = false`, one WARN logged), so a
  bad element shows as "no gas data" rather than a phantom constant. T/P/H are
  unaffected (separate silicon)
- The gas element needs a burn-in period after first power-up — the resistance
  drifts for minutes to hours before it settles
- I2C address: `0x76` (SDO→GND, Bosch primary) or `0x77` (SDO→VCC; Adafruit breakouts default here)
- On ESP32-P4, I2C NACK is reported as `ESP_ERR_INVALID_STATE`
- Before `begin()` (or if it fails) all methods return `ESP_ERR_INVALID_STATE`
- A registered device survives hot-plug: pulling the sensor returns read errors,
  re-inserting recovers automatically (calling `begin()` again re-verifies it)
- No hardware averaging register; reduce noise via oversampling + IIR filter
- **One instance is owned by one task.** The driver is not internally locked;
  share sensor data with other tasks through a queue rather than calling
  `read()` from several tasks on the same instance
- Compensation is single-precision `float` (the Bosch `BME68X_USE_FPU` path) —
  matches the ESP32 hardware FPU, and the arithmetic error is orders of
  magnitude below the sensor's own accuracy; `double` would be slower (emulated)
  with no measurable gain

## Author

Petr Adámek

## License

MIT — see [LICENSE](LICENSE).
