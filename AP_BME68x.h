#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/**
 * @brief Measurement result from a BME680 / BME688.
 */
struct AP_BME68x_Data {
    float temperature;   // °C
    float humidity;      // %RH
    float pressure;      // hPa
    float gasResistance; // Ω — compensated gas reading; 0 when the gas ADC is not valid
    bool  gasValid;      // gas conversion valid AND heater was stable — trust this reading
    bool  heatStable;    // heat_stab_r: heater reached the target temperature this cycle
};

/* ========================================================================== */
/*  AP_BME68x                                                                  */
/*                                                                             */
/*  ESP-IDF driver for the Bosch BME680 / BME688 environmental sensor, using   */
/*  the new I2C master API. Measures temperature, humidity, barometric         */
/*  pressure and gas resistance.                                               */
/*                                                                             */
/*  BME680 and BME688 share chip ID 0x61 but differ in the gas subsystem       */
/*  (run_gas bit, gas registers, resistance compensation). begin() reads the   */
/*  variant_id register and picks the right path automatically; isBME688()     */
/*  reports which one was found.                                               */
/*                                                                             */
/*  Ownership: one instance, one owning task. It is not internally locked —     */
/*  fan sensor data out to other tasks through a queue, not by calling read()  */
/*  from several tasks on the same instance.                                   */
/*                                                                             */
/*  Compensation is single-precision float (the Bosch BME68X_USE_FPU path) —    */
/*  matches the ESP32 hardware FPU; the arithmetic noise is far below the       */
/*  sensor's own accuracy.                                                     */
/*                                                                             */
/*  Usage:                                                                     */
/*    AP_BME68x bme(bus, 0x76);                                                */
/*    bme.begin();                                                             */
/*    AP_BME68x_Data d;                                                        */
/*    if (bme.read(d) == ESP_OK) { ... }                                       */
/* ========================================================================== */

class AP_BME68x
{
public:
    static constexpr uint8_t ADDR_DEFAULT = 0x76;  // SDO pulled low
    static constexpr uint8_t ADDR_ALT     = 0x77;  // SDO pulled high

    static constexpr float NO_SENSOR  = -126.0f;   // sensor not found (no ACK on I2C)
    static constexpr float READ_ERROR = -127.0f;   // sensor found but read failed

    /**
     * @brief Constructor — stores parameters only, does not touch the bus
     *        (never fails / aborts). Call begin() to register and initialize.
     * @param bus     I2C master bus handle (from i2c_new_master_bus)
     * @param address I2C address (0x76 default, 0x77 alternate)
     */
    AP_BME68x(i2c_master_bus_handle_t bus, uint8_t address = ADDR_DEFAULT);

    // Owns an I2C device handle (_dev) — non-copyable to avoid a duplicated
    // handle being used/freed twice.
    AP_BME68x(const AP_BME68x &) = delete;
    AP_BME68x &operator=(const AP_BME68x &) = delete;

    /**
     * @brief Register the device on the I2C bus (once), verify chip ID, detect
     *        the variant, read calibration and apply the default config. Call
     *        before read(). Safe to call again (e.g. recovery) — re-verifies the
     *        sensor.
     * @return ESP_OK on success, ESP_ERR_NOT_FOUND if sensor not present,
     *         or the error from i2c_master_bus_add_device
     */
    esp_err_t begin();

    /**
     * @brief Trigger a forced-mode measurement and return the result.
     *        Blocks for the full conversion time (T+P+H oversampling cycles +
     *        gas heater on-time) plus a short confirmation poll — about 185 ms
     *        with the default config, more with higher oversampling or a longer
     *        heater. Call read() at the task interval — no need to call begin()
     *        again.
     * @param data  Output data struct
     * @return ESP_OK on success. On error every field carries the NO_SENSOR /
     *         READ_ERROR sentinel and gasValid / heatStable are false. Callers
     *         MUST check this return value before scaling or storing the values
     *         — a sentinel put through arithmetic (e.g. Ω→kΩ) silently becomes
     *         a plausible-looking number.
     */
    esp_err_t read(AP_BME68x_Data &data);

    /**
     * @brief Soft-reset sensor (~10 ms). Re-run begin() after reset.
     * @return ESP_OK on success
     */
    esp_err_t reset();

    /**
     * @brief Which variant begin() detected. false = BME680 (gas-low),
     *        true = BME688 (gas-high). Valid only after a successful begin().
     */
    bool isBME688() const { return _isBME688; }

    /**
     * @brief How long read() blocks for the conversion, in ms (heater on-time
     *        included). A confirmation poll may add a little on top. Set by the
     *        last configure() / begin(); size a task period from it if needed.
     */
    uint16_t measurementDurationMs() const { return _measDurMs; }

    /**
     * @brief Set measurement configuration.
     *        Called automatically with defaults in begin(). Call again to change.
     *        Stores the encoded register values; they are applied to the sensor
     *        on the next read() (forced mode rewrites them each cycle).
     *
     * @param tempOs       Temperature oversampling  (0=skip, 1,2,4,8,16)
     * @param pressOs      Pressure oversampling     (0=skip, 1,2,4,8,16)
     * @param humOs        Humidity oversampling     (0=skip, 1,2,4,8,16)
     * @param iirFilter    IIR filter coefficient   (0=off, 1,3,7,15,31,63,127)
     * @param gasHeatTemp  Gas heater target °C     (0 = gas disabled)
     * @param gasHeatDurMs Gas heater duration ms
     * @return ESP_OK on success
     */
    esp_err_t configure(uint8_t  tempOs      = 8,
                        uint8_t  pressOs     = 4,
                        uint8_t  humOs       = 2,
                        uint8_t  iirFilter   = 3,
                        uint16_t gasHeatTemp   = 320,
                        uint16_t gasHeatDurMs  = 150);

private:
    i2c_master_bus_handle_t _bus;
    uint8_t                 _address;
    i2c_master_dev_handle_t _dev = nullptr;
    bool   _isBME688 = false;  // set by begin() from the variant_id register (0xF0)
    int8_t _ambTemp  = 25;     // assumed ambient temperature for heater resistance calculation

    struct _Calib {
        // Temperature
        uint16_t T1;
        int16_t  T2;
        int8_t   T3;
        // Pressure
        uint16_t P1;
        int16_t  P2, P3, P4, P5, P7, P8, P9;
        int8_t   P6, P10;
        // Humidity
        uint16_t H1, H2;
        int8_t   H3, H4, H5, H7;
        uint8_t  H6;
        // Gas / heater
        int8_t   G1, G3;
        int16_t  G2;
        uint8_t  resHeatRange;
        int8_t   resHeatVal;
        int8_t   rangeSWErr;
    } _cal = {};

    bool     _gasEnabled  = false;
    uint8_t  _ctrlHum     = 0;
    uint8_t  _ctrlMeas    = 0;
    uint8_t  _config      = 0;
    uint8_t  _resHeat0    = 0;
    uint8_t  _gasWait0    = 0;
    uint16_t _gasHeatTemp = 0;   // gas heater target [°C] (0 = gas disabled) — for per-cycle _resHeat0
    uint16_t _measDurMs   = 0;   // precomputed forced-mode measurement duration [ms]
    bool     _gasFaultLogged = false;   // one-shot WARN latch for a dead gas element

    esp_err_t _readReg(uint8_t reg, uint8_t *data, size_t len);
    esp_err_t _writeReg(uint8_t reg, uint8_t val);
    esp_err_t _readCalibration();

    // Full forced-mode cycle; fills data on success. read() wraps it and maps
    // any error to the sentinel values via _errorValue().
    esp_err_t _measure(AP_BME68x_Data &data);

    // Maps an esp_err_t to NO_SENSOR (disconnect/no ACK) or READ_ERROR.
    static float _errorValue(esp_err_t err);

    float _compensateTemp(uint32_t adcTemp, int32_t &tFine);
    float _compensatePress(uint32_t adcPres, int32_t tFine);
    float _compensateHum(uint16_t adcHum, int32_t tFine);
    float _compensateGas(uint16_t adcGas, uint8_t gasRange) const;      // BME680 (gas-low)
    float _compensateGasHigh(uint16_t adcGas, uint8_t gasRange) const;  // BME688 (gas-high)

    uint8_t  _encodeOsrs(uint8_t os);
    uint8_t  _encodeFilter(uint8_t filter);
    uint8_t  _calcHeaterRes(uint16_t targetTemp);
    uint8_t  _encodeHeaterDur(uint16_t durMs);
    uint16_t _decodeHeaterDurMs(uint8_t reg) const;   // inverse of _encodeHeaterDur()
};
