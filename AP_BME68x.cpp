#include "AP_BME68x.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "AP_BME68x";

// ---------------------------------------------------------------------------
// Register addresses
// ---------------------------------------------------------------------------
static constexpr uint8_t REG_CHIP_ID    = 0xD0;
static constexpr uint8_t CHIP_ID_VAL    = 0x61;  // shared by BME680 and BME688
static constexpr uint8_t REG_VARIANT_ID = 0xF0;  // 0x00 = BME680 (gas low), 0x01 = BME688 (gas high)
static constexpr uint8_t VARIANT_BME688 = 0x01;
static constexpr uint8_t REG_RESET      = 0xE0;
static constexpr uint8_t RESET_CMD      = 0xB6;
static constexpr uint8_t REG_CTRL_HUM   = 0x72;
static constexpr uint8_t REG_CTRL_GAS_0 = 0x70;
static constexpr uint8_t REG_CTRL_GAS_1 = 0x71;
static constexpr uint8_t REG_CTRL_MEAS  = 0x74;
static constexpr uint8_t REG_CONFIG     = 0x75;
static constexpr uint8_t REG_RES_HEAT_0 = 0x5A;
static constexpr uint8_t REG_GAS_WAIT_0 = 0x64;
static constexpr uint8_t REG_STATUS_0   = 0x1D;
static constexpr uint8_t REG_DATA_START = 0x1F;

// Control / status bit masks
static constexpr uint8_t MODE_MSK       = 0x03;  // ctrl_meas mode bits
static constexpr uint8_t MODE_SLEEP     = 0x00;  // ctrl_meas mode = sleep
static constexpr uint8_t MODE_FORCED    = 0x01;  // ctrl_meas mode = forced
static constexpr uint8_t HEAT_OFF_BIT   = 0x08;  // ctrl_gas_0 heat_off (1 = heater disabled)
static constexpr uint8_t RUN_GAS_LOW    = 0x10;  // ctrl_gas_1 run_gas for BME680 (mask 0x30, val 1<<4)
static constexpr uint8_t RUN_GAS_HIGH   = 0x20;  // ctrl_gas_1 run_gas for BME688 (mask 0x30, val 2<<4)
static constexpr uint8_t STATUS_NEW_DATA = 0x80; // status_0 new_data_0
static constexpr uint8_t GAS_VALID_BIT   = 0x20; // gas_r_lsb gas_valid_r
static constexpr uint8_t HEAT_STAB_BIT   = 0x10; // gas_r_lsb heat_stab_r

// gas_r register offsets inside the 0x1F.. data block: BME680 at 0x2A/0x2B,
// BME688 at 0x2C/0x2D.
static constexpr uint8_t GAS_OFF_LOW    = 0x2A - REG_DATA_START;  // = 11
static constexpr uint8_t GAS_OFF_HIGH   = 0x2C - REG_DATA_START;  // = 13
static constexpr uint8_t DATA_LEN       = 0x2D - REG_DATA_START + 1;  // = 15

// Calibration register banks (extra block starts at 0x00: [2] = res_heat_range
// bits [5:4], [4] = range_sw_err bits [7:4]).
static constexpr uint8_t REG_CAL1      = 0x89;  // 25 bytes: 0x89..0xA1
static constexpr uint8_t REG_CAL2      = 0xE1;  // 14 bytes: 0xE1..0xEE
static constexpr uint8_t REG_CAL_EXTRA = 0x00;  // 5 bytes: 0x00..0x04

static constexpr int I2C_TIMEOUT_MS   = 100;  // per register transfer
static constexpr int PROBE_TIMEOUT_MS = 50;   // address probe after a failed read()

// Ambient temperature estimate used for the heater resistance is clamped to
// the sensor's operating range (also keeps the float -> int8_t conversion defined).
static constexpr float AMB_TEMP_MIN = -40.0f;
static constexpr float AMB_TEMP_MAX = 85.0f;

// Per-range correction factors for the gas-low (BME680) resistance formula —
// Bosch BME68x reference (calc_gas_resistance_low, float).
static const float GAS_LOW_K1[16] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, -0.8f,
    0.0f, 0.0f, -0.2f, -0.5f, 0.0f, -1.0f, 0.0f, 0.0f
};
static const float GAS_LOW_K2[16] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.7f, 0.0f, -0.8f,
    -0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

// ---------------------------------------------------------------------------
// Constructor — stores parameters only, never touches the bus / aborts.
// ---------------------------------------------------------------------------
AP_BME68x::AP_BME68x(i2c_master_bus_handle_t bus, uint8_t address)
    : _bus(bus), _address(address)
{
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
esp_err_t AP_BME68x::begin()
{
    // Register the device on the bus once (idempotent).
    const bool firstInit = (_dev == nullptr);
    if (_dev == nullptr) {
        i2c_device_config_t cfg = {};
        cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        cfg.device_address  = _address;
        cfg.scl_speed_hz    = 100000;
        esp_err_t add = i2c_master_bus_add_device(_bus, &cfg, &_dev);
        if (add != ESP_OK) {
            _dev = nullptr;
            ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(add));
            return add;
        }
    }

    // On the first init, soft-reset the sensor to a known-good state — an MCU
    // reboot alone does not reset the sensor, so any latched mode / gas state
    // from a previous firmware session would otherwise persist. Skipped on a
    // recovery re-begin() (called from the read loop) so it stays lightweight.
    if (firstInit) {
        reset();  // return ignored — the chip-ID read below is the real gate
    }

    // Verify + calibration runs on every begin() — supports recovery re-begin.
    uint8_t id = 0;
    esp_err_t ret = _readReg(REG_CHIP_ID, &id, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No ACK on I2C (sensor not connected?)");
        return ESP_ERR_NOT_FOUND;
    }
    if (id != CHIP_ID_VAL) {
        ESP_LOGE(TAG, "Wrong chip ID: 0x%02X (expected 0x%02X)", id, CHIP_ID_VAL);
        return ESP_ERR_NOT_FOUND;
    }

    // Chip ID 0x61 is shared by BME680 and BME688 — the variant register decides
    // the gas subsystem (run_gas bit position, gas_r register location and the
    // resistance compensation formula all differ).
    uint8_t variant = 0;
    ret = _readReg(REG_VARIANT_ID, &variant, 1);
    if (ret != ESP_OK) return ret;
    _isBME688 = (variant == VARIANT_BME688);

    ret = _readCalibration();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data");
        return ret;
    }

    ESP_LOGI(TAG, "%s found, calibration loaded", _isBME688 ? "BME688" : "BME680");
    return configure();
}

esp_err_t AP_BME68x::read(AP_BME68x_Data &data)
{
    data = {};
    esp_err_t ret = _measure(data);
    if (ret != ESP_OK) {
        // Single point of failure handling: classify disconnect vs read error
        // uniformly across every step of the sequence (no duplicated logic).
        ret = _classifyError(ret);
        const float sentinel = (ret == ESP_ERR_NOT_FOUND || _dev == nullptr)
                               ? NO_SENSOR : READ_ERROR;
        data.temperature   = sentinel;
        data.humidity      = sentinel;
        data.pressure      = sentinel;
        data.gasResistance = sentinel;
        data.gasValid      = false;
        data.heatStable    = false;
    }
    return ret;
}

esp_err_t AP_BME68x::_measure(AP_BME68x_Data &data)
{
    // Config registers (ctrl_hum/config/ctrl_gas/res_heat/gas_wait) only latch
    // in sleep mode. A completed forced measurement leaves the sensor asleep, so
    // normally it already is — but a prior timed-out read() could have left one
    // running. Put it back to sleep first so the sequence is deterministic
    // regardless of prior state.
    uint8_t ctrlMeas = 0;
    esp_err_t ret = _readReg(REG_CTRL_MEAS, &ctrlMeas, 1);
    if (ret != ESP_OK) return ret;
    if ((ctrlMeas & MODE_MSK) != MODE_SLEEP) {
        ret = _writeReg(REG_CTRL_MEAS, (uint8_t)(ctrlMeas & ~MODE_MSK));
        if (ret != ESP_OK) return ret;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Re-apply the full config every cycle. This is deliberate (not redundant):
    // after a hot-plug the sensor resets its registers to defaults, so rewriting
    // them here recovers measurement without an explicit begin() call.
    ret = _writeReg(REG_CTRL_HUM, _ctrlHum);
    if (ret != ESP_OK) return ret;
    ret = _writeReg(REG_CONFIG, _config);
    if (ret != ESP_OK) return ret;

    // Heater on/off (ctrl_gas_0.heat_off) and the gas run bit (ctrl_gas_1) are
    // set together so the gas-enable state is never split across a reset.
    ret = _writeReg(REG_CTRL_GAS_0, _gasEnabled ? 0x00 : HEAT_OFF_BIT);
    if (ret != ESP_OK) return ret;

    if (_gasEnabled) {
        // Recompute the heater resistance from the latest ambient estimate every
        // cycle. A value fixed at begin() drifts as the room temperature moves,
        // which shows up as heat_stab_r dropping out (and the gas reading going
        // stale on consumers that gate on it).
        ret = _writeReg(REG_RES_HEAT_0, _calcHeaterRes(_gasHeatTemp));
        if (ret != ESP_OK) return ret;
        ret = _writeReg(REG_GAS_WAIT_0, _gasWait0);
        if (ret != ESP_OK) return ret;
        // run_gas sits in ctrl_gas_1 bits [5:4]: value 1 (0x10) on BME680,
        // value 2 (0x20) on BME688. Writing the wrong one leaves the gas
        // measurement disabled — the ADC then reads 0.
        ret = _writeReg(REG_CTRL_GAS_1, _isBME688 ? RUN_GAS_HIGH : RUN_GAS_LOW);
        if (ret != ESP_OK) return ret;
    } else {
        ret = _writeReg(REG_CTRL_GAS_1, 0x00);  // run_gas = 0
        if (ret != ESP_OK) return ret;
    }

    // Start forced mode (returns to sleep automatically when the cycle finishes)
    ret = _writeReg(REG_CTRL_MEAS, _ctrlMeas | MODE_FORCED);
    if (ret != ESP_OK) return ret;

    // The forced cycle runs T -> P -> H -> gas heater -> gas ADC. Wait the full
    // precomputed conversion time (TPH oversampling cycles + fixed overhead +
    // gas heater on-time) before polling new_data — the Bosch BME68x reference
    // flow (get_meas_dur + delay, then poll). Duration computed in configure().
    vTaskDelay(pdMS_TO_TICKS(_measDurMs));

    // Bounded confirmation poll on new_data_0 — slack for a cold heater, bus
    // contention or CPU frequency scaling making the cycle run long.
    uint8_t status = 0;
    for (int i = 0; i < 20; i++) {
        ret = _readReg(REG_STATUS_0, &status, 1);
        if (ret != ESP_OK) return ret;
        if (status & STATUS_NEW_DATA) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!(status & STATUS_NEW_DATA)) {
        ESP_LOGW(TAG, "Measurement timeout (status=0x%02X)", status);
        return ESP_ERR_TIMEOUT;
    }

    // Read raw ADC data: registers 0x1F..0x2D (covers both the BME680 and the
    // BME688 gas_r location).
    uint8_t buf[DATA_LEN] = {};
    ret = _readReg(REG_DATA_START, buf, DATA_LEN);
    if (ret != ESP_OK) return ret;

    // Extract ADC values. gas_r is at a different offset per variant.
    const uint8_t g = _isBME688 ? GAS_OFF_HIGH : GAS_OFF_LOW;   // gas_r_msb offset
    uint32_t adcPres = ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | (buf[2] >> 4);
    uint32_t adcTemp = ((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | (buf[5] >> 4);
    uint16_t adcHum  = ((uint16_t)buf[6] << 8)  |  buf[7];
    uint16_t adcGas  = ((uint16_t)buf[g] << 2)  | (buf[g + 1] >> 6);
    uint8_t  gasRange = buf[g + 1] & 0x0F;
    bool     gasValid = (buf[g + 1] & GAS_VALID_BIT) != 0;
    bool     heatStab = (buf[g + 1] & HEAT_STAB_BIT) != 0;

    // Reject a physically-impossible "valid" gas reading: a degraded / dead gas
    // element can still assert gas_valid_r + heat_stab_r while the ADC returns
    // 0, which the compensation formula turns into a plausible-looking constant
    // (~43 kOhm). A functional conversion never yields a zero ADC count.
    if (gasValid && adcGas == 0) {
        gasValid = false;
        if (!_gasFaultLogged) {
            ESP_LOGW(TAG, "Gas element unresponsive (adc_gas=0 with valid+stab set) — gas reported invalid");
            _gasFaultLogged = true;
        }
    } else if (gasValid) {
        _gasFaultLogged = false;
    }

    // Compensate
    int32_t tFine = 0;
    data.temperature   = _compensateTemp(adcTemp, tFine);
    data.pressure      = _compensatePress(adcPres, tFine);
    data.humidity      = _compensateHum(adcHum, tFine);
    // gasResistance is filled whenever the gas ADC is valid, so a caller can
    // publish a best-effort value while the heater is still settling. gasValid
    // is the strict flag (ADC valid AND heater stable); heatStable exposes the
    // heater bit on its own.
    data.gasResistance = (_gasEnabled && gasValid)
                         ? (_isBME688 ? _compensateGasHigh(adcGas, gasRange)
                                      : _compensateGas(adcGas, gasRange))
                         : 0.0f;
    data.gasValid      = _gasEnabled && gasValid && heatStab;
    data.heatStable    = heatStab;

    // Update ambient temperature estimate for next heater calculation
    float amb = data.temperature;
    if (amb < AMB_TEMP_MIN) amb = AMB_TEMP_MIN;
    if (amb > AMB_TEMP_MAX) amb = AMB_TEMP_MAX;
    _ambTemp = (int8_t)amb;

    return ESP_OK;
}

esp_err_t AP_BME68x::reset()
{
    esp_err_t ret = _writeReg(REG_RESET, RESET_CMD);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ret;
}

esp_err_t AP_BME68x::configure(uint8_t tempOs, uint8_t pressOs, uint8_t humOs,
                                uint8_t iirFilter, uint16_t gasHeatTemp, uint16_t gasHeatDurMs)
{
    const uint8_t osrsT = _encodeOsrs(tempOs);
    const uint8_t osrsP = _encodeOsrs(pressOs);
    const uint8_t osrsH = _encodeOsrs(humOs);

    _ctrlHum  = osrsH & 0x07;
    _ctrlMeas = (osrsT << 5) | (osrsP << 2);
    _config   = (_encodeFilter(iirFilter) << 2) & 0x1C;

    _gasEnabled  = (gasHeatTemp > 0);
    _gasHeatTemp = _gasEnabled ? gasHeatTemp : 0;
    // The heater resistance (res_heat_0) is computed per cycle in _measure()
    // from the latest ambient temperature - only the on-time is fixed here.
    if (_gasEnabled) {
        _gasWait0 = _encodeHeaterDur(gasHeatDurMs);
    }

    // Precompute the forced-mode measurement duration (Bosch BME68x timing
    // model): TPH conversion cycles * 1963 us + fixed switching / gas-conversion
    // overhead + 1 ms wake-up, plus the quantised gas heater on-time. read()
    // waits this before polling so the gas phase is always finished when the
    // data registers are read.
    static const uint8_t OSR_CYCLES[6] = { 0, 1, 2, 4, 8, 16 };
    uint32_t durUs = ((uint32_t)OSR_CYCLES[osrsT] + OSR_CYCLES[osrsP] + OSR_CYCLES[osrsH]) * 1963u
                     + 477u * 4u    // TPH switching duration
                     + 477u * 5u    // gas measurement duration
                     + 500u;        // round to the nearest ms
    uint32_t durMs = durUs / 1000u + 1u;   // + 1 ms wake-up
    if (_gasEnabled) durMs += _decodeHeaterDurMs(_gasWait0);
    _measDurMs = (uint16_t)durMs;

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Private — I2C helpers
// ---------------------------------------------------------------------------
esp_err_t AP_BME68x::_readReg(uint8_t reg, uint8_t *data, size_t len)
{
    if (_dev == nullptr) return ESP_ERR_INVALID_STATE;  // begin() not called
    return i2c_master_transmit_receive(_dev, &reg, 1, data, len, I2C_TIMEOUT_MS);
}

esp_err_t AP_BME68x::_writeReg(uint8_t reg, uint8_t val)
{
    if (_dev == nullptr) return ESP_ERR_INVALID_STATE;  // begin() not called
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(_dev, buf, 2, I2C_TIMEOUT_MS);
}

esp_err_t AP_BME68x::_classifyError(esp_err_t err) const
{
    // The error code of a failed I2C transfer cannot tell a missing sensor
    // from a failed read: it differs between ESP-IDF versions, and newer
    // releases report a NACK and a bus timeout with the same code
    // (ESP_ERR_INVALID_RESPONSE). A direct address probe can. Any probe
    // failure means the sensor is unreachable: a NACK (bus pulled up, nobody
    // answers) or a bus timeout - breakout modules usually carry the I2C
    // pull-ups, so with the module unplugged the lines stay low and the
    // hardware times out instead of seeing a NACK. Runs only on the error path.
    if (_dev == nullptr) return err;  // ESP_ERR_INVALID_STATE - begin() not called
    if (i2c_master_probe(_bus, _address, PROBE_TIMEOUT_MS) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    return err;  // sensor answers - bus error or measurement timeout
}

// ---------------------------------------------------------------------------
// Private — Calibration
// ---------------------------------------------------------------------------
esp_err_t AP_BME68x::_readCalibration()
{
    uint8_t c1[25] = {};
    uint8_t c2[14] = {};
    uint8_t extra[5] = {};

    esp_err_t ret = _readReg(REG_CAL1, c1, 25);
    if (ret != ESP_OK) return ret;
    ret = _readReg(REG_CAL2, c2, 14);
    if (ret != ESP_OK) return ret;
    ret = _readReg(REG_CAL_EXTRA, extra, 5);  // 0x00..0x04
    if (ret != ESP_OK) return ret;

    // Temperature — c1 offsets from 0x89
    _cal.T2 = (int16_t)((uint16_t)c1[2] << 8 | c1[1]);  // 0x8B:0x8A
    _cal.T3 = (int8_t)c1[3];                              // 0x8C
    // T1 is in c2
    _cal.T1 = (uint16_t)((uint16_t)c2[9] << 8 | c2[8]);  // 0xEA:0xE9

    // Pressure
    _cal.P1 = (uint16_t)((uint16_t)c1[6]  << 8 | c1[5]);   // 0x8F:0x8E
    _cal.P2 = (int16_t)((uint16_t)c1[8]   << 8 | c1[7]);   // 0x91:0x90
    _cal.P3 = (int8_t)c1[9];                                 // 0x92
    _cal.P4 = (int16_t)((uint16_t)c1[12]  << 8 | c1[11]);  // 0x95:0x94
    _cal.P5 = (int16_t)((uint16_t)c1[14]  << 8 | c1[13]);  // 0x97:0x96
    _cal.P7 = (int8_t)c1[15];                                // 0x98
    _cal.P6 = (int8_t)c1[16];                                // 0x99
    _cal.P8 = (int16_t)((uint16_t)c1[20]  << 8 | c1[19]);  // 0x9D:0x9C
    _cal.P9 = (int16_t)((uint16_t)c1[22]  << 8 | c1[21]);  // 0x9F:0x9E
    _cal.P10 = (int8_t)c1[23];                               // 0xA0

    // Humidity (split across c2[0..2])
    _cal.H2 = (uint16_t)((uint16_t)c2[0] << 4 | (c2[1] >> 4));   // 0xE1 MSB, 0xE2[7:4]
    _cal.H1 = (uint16_t)((uint16_t)c2[2] << 4 | (c2[1] & 0x0F)); // 0xE3 MSB, 0xE2[3:0]
    _cal.H3 = (int8_t)c2[3];   // 0xE4
    _cal.H4 = (int8_t)c2[4];   // 0xE5
    _cal.H5 = (int8_t)c2[5];   // 0xE6
    _cal.H6 = c2[6];            // 0xE7
    _cal.H7 = (int8_t)c2[7];   // 0xE8

    // Gas
    _cal.G2 = (int16_t)((uint16_t)c2[11] << 8 | c2[10]);  // 0xEC:0xEB
    _cal.G1 = (int8_t)c2[12];  // 0xED
    _cal.G3 = (int8_t)c2[13];  // 0xEE

    // Extra calibration (registers 0x00..0x04)
    _cal.resHeatVal   = (int8_t)extra[0];                     // 0x00
    _cal.resHeatRange = (extra[2] & 0x30) >> 4;               // 0x02 bits [5:4]
    _cal.rangeSWErr   = (int8_t)(extra[4] & 0xF0) / 16;       // 0x04 bits [7:4], signed (−8..7)

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Private — Compensation (float, from Bosch BME680 reference implementation)
// ---------------------------------------------------------------------------
float AP_BME68x::_compensateTemp(uint32_t adcTemp, int32_t &tFine) const
{
    float var1 = ((float)adcTemp / 16384.0f - (float)_cal.T1 / 1024.0f) * (float)_cal.T2;
    float var2 = (float)adcTemp / 131072.0f - (float)_cal.T1 / 8192.0f;
    var2 = var2 * var2 * (float)_cal.T3 * 16.0f;
    tFine = (int32_t)(var1 + var2);
    return (var1 + var2) / 5120.0f;
}

float AP_BME68x::_compensatePress(uint32_t adcPres, int32_t tFine) const
{
    float var1 = ((float)tFine / 2.0f) - 64000.0f;
    float var2 = var1 * var1 * (float)_cal.P6 / 131072.0f;
    var2 = var2 + var1 * (float)_cal.P5 * 2.0f;
    var2 = var2 / 4.0f + (float)_cal.P4 * 65536.0f;
    var1 = ((float)_cal.P3 * var1 * var1 / 16384.0f + (float)_cal.P2 * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * (float)_cal.P1;
    if (var1 == 0.0f) return 0.0f;

    float pressure = 1048576.0f - (float)adcPres;
    pressure = ((pressure - var2 / 4096.0f) * 6250.0f) / var1;
    var1 = (float)_cal.P9 * pressure * pressure / 2147483648.0f;
    var2 = pressure * (float)_cal.P8 / 32768.0f;
    float var3 = (pressure / 256.0f) * (pressure / 256.0f) * (pressure / 256.0f)
                 * (float)_cal.P10 / 131072.0f;
    pressure = pressure + (var1 + var2 + var3 + (float)_cal.P7 * 128.0f) / 16.0f;
    return pressure / 100.0f;  // Pa → hPa
}

float AP_BME68x::_compensateHum(uint16_t adcHum, int32_t tFine) const
{
    float temp = (float)tFine / 5120.0f;
    float var1 = (float)adcHum
                 - ((float)_cal.H1 * 16.0f + (float)_cal.H3 / 2.0f * temp);
    float var2 = var1 * ((float)_cal.H2 / 262144.0f
                 * (1.0f + (float)_cal.H4 / 16384.0f * temp
                    + (float)_cal.H5 / 1048576.0f * temp * temp));
    float var3 = (float)_cal.H6 / 16384.0f;
    float var4 = (float)_cal.H7 / 2097152.0f;
    float hum  = var2 + (var3 + var4 * temp) * var2 * var2;
    if (hum > 100.0f) hum = 100.0f;
    if (hum < 0.0f)   hum = 0.0f;
    return hum;
}

float AP_BME68x::_compensateGas(uint16_t adcGas, uint8_t gasRange) const
{
    // BME680 (gas-low variant) resistance in Ω — Bosch BME68x reference
    // (calc_gas_resistance_low, float). Uses range_sw_err (signed).
    const float rangeF = (float)(1u << gasRange);
    const float var1 = 1340.0f + 5.0f * (float)_cal.rangeSWErr;
    const float var2 = var1 * (1.0f + GAS_LOW_K1[gasRange] / 100.0f);
    const float var3 = 1.0f + GAS_LOW_K2[gasRange] / 100.0f;
    return 1.0f / (var3 * 0.000000125f * rangeF * (((float)adcGas - 512.0f) / var2 + 1.0f));
}

float AP_BME68x::_compensateGasHigh(uint16_t adcGas, uint8_t gasRange) const
{
    // BME688 (gas-high variant) resistance in Ω — Bosch BME68x reference
    // (calc_gas_resistance_high, float). No calibration coefficients involved.
    const uint32_t var1 = 262144u >> gasRange;
    const int32_t  var2 = 4096 + ((int32_t)adcGas - 512) * 3;
    return 1000000.0f * (float)var1 / (float)var2;
}

// ---------------------------------------------------------------------------
// Private — Config helpers
// ---------------------------------------------------------------------------
uint8_t AP_BME68x::_encodeOsrs(uint8_t os)
{
    if (os == 0)  return 0;
    if (os <= 1)  return 1;
    if (os <= 2)  return 2;
    if (os <= 4)  return 3;
    if (os <= 8)  return 4;
    return 5;  // 16x
}

uint8_t AP_BME68x::_encodeFilter(uint8_t filter)
{
    if (filter == 0)  return 0;
    if (filter <= 1)  return 1;
    if (filter <= 3)  return 2;
    if (filter <= 7)  return 3;
    if (filter <= 15) return 4;
    if (filter <= 31) return 5;
    if (filter <= 63) return 6;
    return 7;  // 127
}

uint8_t AP_BME68x::_calcHeaterRes(uint16_t targetTemp) const
{
    if (targetTemp > 400) targetTemp = 400;
    float var1 = ((float)_cal.G1 / 16.0f) + 49.0f;
    float var2 = ((float)_cal.G2 / 32768.0f * 0.0005f) + 0.00235f;
    float var3 = (float)_cal.G3 / 1024.0f;
    float var4 = var1 * (1.0f + (var2 * (float)targetTemp));
    float var5 = var4 + (var3 * (float)_ambTemp);
    return (uint8_t)(3.4f * ((var5 * (4.0f / (4.0f + (float)_cal.resHeatRange))
                     * (1.0f / (1.0f + (float)_cal.resHeatVal * 0.002f))) - 25.0f));
}

uint8_t AP_BME68x::_encodeHeaterDur(uint16_t durMs)
{
    uint8_t factor = 0;
    while (durMs > 63 && factor < 3) {
        durMs /= 4;
        factor++;
    }
    if (durMs > 63) durMs = 63;
    return (uint8_t)(durMs | (factor << 6));
}

uint16_t AP_BME68x::_decodeHeaterDurMs(uint8_t reg)
{
    // gas_wait_0: on-time = value (bits [5:0]) * multiplier (bits [7:6] = x1/x4/x16/x64).
    static const uint16_t MULT[4] = { 1, 4, 16, 64 };
    return (uint16_t)(reg & 0x3F) * MULT[(reg >> 6) & 0x03];
}
