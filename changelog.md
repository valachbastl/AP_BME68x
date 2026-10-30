# Changelog

## [1.1.0] - 2026-10-30

### Fixed
- Rozlišení „senzor nenalezen“ / „chyba čtení“ v `read()` bylo obrácené.
  Sentinel se určoval z kódu chyby I2C přenosu: `ESP_ERR_TIMEOUT` → `NO_SENSOR`,
  ostatní → `READ_ERROR`. Nové ESP-IDF ale při NACK (odpojený senzor) i při
  timeoutu sběrnice vrací `ESP_ERR_INVALID_RESPONSE` → odpojený senzor hlásil
  `READ_ERROR`; naopak `ESP_ERR_TIMEOUT` vrací jen vlastní timeout měření
  (senzor odpovídá, data nepřišla) → hlásil `NO_SENSOR`. Teď se po selhání
  `read()` jednou otestuje adresa senzoru (`i2c_master_probe`): neúspěch
  (NACK nebo timeout sběrnice — moduly mívají pull-upy na sobě, po vytažení
  zůstanou linky dole a místo NACK přijde timeout) = `NO_SENSOR`, senzor
  odpovídá = `READ_ERROR`. Nezávislé na verzi ESP-IDF, probe běží jen
  v chybové větvi.
- `CMakeLists.txt`: `REQUIRES` uváděl neexistující komponentu `esp_log` (komponenta
  logu se v ESP-IDF jmenuje `log`) → čistý ESP-IDF build (`idf.py`) selhal při
  konfiguraci na „Failed to resolve component“. Opraveno na `log`.
- `idf_component.yml`: minimální ESP-IDF `>=5.0.0` → `>=5.2.0` (nový I2C master
  driver `driver/i2c_master.h` existuje od 5.2).

### Changed
- `read()` vrací při nenalezeném senzoru `ESP_ERR_NOT_FOUND` (stejně jako
  `begin()`), dřív kód chyby I2C přenosu závislý na verzi ESP-IDF. Ostatní chyby
  beze změny.
- Odhad okolní teploty pro výpočet odporu topení se omezuje na pracovní rozsah
  senzoru (−40…85 °C) — převod `float` → `int8_t` mimo rozsah by byl nedefinovaný.
- Odstraněn nepoužitý výpočet odporu topení v `configure()` (hodnota se stejně
  přepočítává v každém cyklu měření) a člen `_resHeat0`. Chování beze změny.
- README: opravena poznámka o stavu před `begin()` (`configure()` jen ukládá
  nastavení, lze volat kdykoli); odstraněna zastaralá poznámka o kódu NACK na P4.

**Otestováno na HW** 2026-10-30 — ESP32 (LaskaKit ESPlan + BME688 na 0x77, ESP-IDF 6.1):
běžné měření T/P/H/plyn beze změny; vytažení senzoru za běhu → `NO_SENSOR` (`-126`),
zpětné zasunutí → měření se samo obnoví; start bez senzoru → `begin()` `ESP_ERR_NOT_FOUND`,
`read()` `NO_SENSOR`.

## [1.0.0] - 2026-09-07

První veřejné vydání. Ověřeno na reálném HW (LaskaKit ESPlan + BME688 na 0x77).

### Added
- Inicializace `begin()` – zaregistruje zařízení na I2C sběrnici, ověří chip ID
  (0x61), detekuje variantu, načte kalibraci a aplikuje výchozí konfiguraci.
  Vrací `esp_err_t` (konstruktor nesahá na hardware ani neabortuje aplikaci).
  Idempotentní; opakované volání re-ověří senzor (recovery). Při prvním volání
  provede soft-reset senzoru (`0xE0 = 0xB6`) – MCU reboot senzor nevynuluje.
- **Automatická detekce BME680 vs BME688.** Sdílejí chip ID `0x61`, ale liší se
  v plynovém subsystému. `begin()` čte registr `variant_id` (`0xF0`: `0x00` =
  BME680 / gas-low, `0x01` = BME688 / gas-high) a `read()` podle toho volí:
  - `run_gas` bit v `ctrl_gas_1` – `0x10` (BME680) vs `0x20` (BME688),
  - umístění `gas_r` registrů – `0x2A/0x2B` vs `0x2C/0x2D`,
  - vzorec kompenzace odporu – `calc_gas_resistance_low` (BME680) vs
    `calc_gas_resistance_high` (BME688), obojí float dle Bosch BME68x reference.

  `isBME688()` vrací detekovanou variantu. Teplota/vlhkost/tlak jsou u obou
  čipů shodné.
- Vynucené měření `read(AP_BME68x_Data&)` – teplota [°C], vlhkost [%RH], tlak
  [hPa], odpor plynu [Ω] + příznaky `gasValid` (gas ADC platný **i** heater
  ustálený) a `heatStable` (`heat_stab_r` samostatně). Float kompenzace dle
  Bosch BME68x reference (`range_sw_err` znaménkově). Doba měření se počítá
  v `configure()` dle Bosch timing modelu (TPH konverzní cykly + fixní režie +
  doba ohřevu heateru); `read()` ji počká a pak provede omezený potvrzovací
  poll na `new_data_0`. Blokuje ~185 ms s výchozí konfigurací.
  `measurementDurationMs()` tuto dobu vrací (pro nastavení periody tasku).
- `_measure()` před přepisem konfiguračních registrů ověří, že je senzor ve
  sleep módu (a případně ho tam uspí) – konfigurace latchuje jen ve sleep;
  normálně po dokončeném forced měření senzor spí, tohle pokryje i stav po
  timeoutovaném `read()`. Heater on/off (`ctrl_gas_0.heat_off`) se nastavuje
  spolu s `run_gas`, aby gas-enable stav nebyl rozdělený.
- `_resHeat0` (odpor heateru) se přepočítává z aktuálního odhadu okolní teploty
  každý cyklus, ne jen jednou při konfiguraci.
- Guard na `adc_gas == 0` s nastavenými `gas_valid_r`/`heat_stab_r` – degradovaný
  plynový element umí takové čtení ohlásit jako „platné", kompenzace by z něj
  udělala věrohodnou konstantu. Čtení se zahodí (`gasValid = false`) + jednou
  zaloguje WARN.
- Konfigurace měření `configure()` – oversampling (T/P/H), IIR filtr, teplota a
  doba ohřevu plynového senzoru. Voláno s výchozími hodnotami v `begin()`.
- Soft-reset `reset()`.
- Sentinel konstanty `NO_SENSOR = -126.0f` (senzor nenalezen / no ACK) a
  `READ_ERROR = -127.0f` (senzor nalezen, ale čtení selhalo) jako veřejné členy.
  Návratový `esp_err_t` z `read()` se **musí** zkontrolovat před škálováním /
  uložením hodnot – sentinel protažený aritmetikou (např. Ω→kΩ) se tiše změní
  na věrohodné číslo.
- Před `begin()` (nebo když selže) vrací I2C funkce `ESP_ERR_INVALID_STATE`.
- ESP-IDF component balení (`CMakeLists.txt` + `idf_component.yml`).
- Třída je nekopírovatelná (`= delete` copy-ctor/assignment) – vlastní I2C
  handle, zabráněno dvojímu použití/uvolnění.
