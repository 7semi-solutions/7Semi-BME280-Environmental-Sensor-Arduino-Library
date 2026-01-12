# 7Semi-BME690-Arduino-Library

This Arduino library provides support for the **7Semi BME690 Sensor Module**, a powerful 3-in-1 environmental sensor capable of measuring **temperature**, **humidity**, and **barometric pressure** via I2C. It is ideal for air quality monitoring, weather stations, and smart environmental IoT systems.

![Arduino](https://img.shields.io/badge/platform-arduino-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![Status](https://img.shields.io/badge/status-active-brightgreen.svg)

---

## Hardware Required

- 7Semi BME690 Sensor Module  
- Arduino-compatible board  # 7Semi BME280 Arduino Library

Arduino-compatible wrapper library for the **Bosch Sensortec BME280** environmental sensor.

This library uses the official Bosch Sensortec BME280 driver internally and provides an easy-to-use Arduino-style interface for **I2C** and **SPI**.

---

## Features

- ✅ Supports **I2C** and **SPI**
- ✅ Reads **Temperature (°C)**, **Pressure (hPa)**, **Humidity (%RH)**
- ✅ Includes **low-power forced measurement mode**
- ✅ Supports configuration:
  - Oversampling (Temp / Pressure / Humidity)
  - IIR filter
  - Standby duration (Normal mode)
- ✅ Arduino Library Manager compatible structure

---

## Supported Hardware

- Bosch Sensortec **BME280**
- Compatible breakout modules (3.3V / 5V depending on module regulator)

---

## Installation

### Arduino IDE (ZIP install)
1. Download the library as a `.zip`
2. Open Arduino IDE → **Sketch → Include Library → Add .ZIP Library**
3. Select the downloaded zip
4. Restart Arduino IDE

### Manual install
1. Copy this library folder into your Arduino libraries directory:

- Windows: `Documents/Arduino/libraries/`
- Linux: `~/Arduino/libraries/`
- macOS: `~/Documents/Arduino/libraries/`

---

## Wiring

### I2C Wiring

| BME280 Pin | Arduino Pin |
|----------|------------|
| VCC      | 3.3V / 5V  |
| GND      | GND        |
| SDA      | SDA        |
| SCL      | SCL        |

Default I2C address:
- `0x76` (most modules)
- `0x77` (alternate, depends on SDO pin)

---

### SPI Wiring

| BME280 Pin | Arduino Pin |
|----------|------------|
| VCC      | 3.3V / 5V  |
| GND      | GND        |
| CS       | Any GPIO (example: 10) |
| SCK      | SCK        |
| MOSI (SDI)    | MOSI       |
| MISO (SDO)    | MISO       |

---

## Major Functions (Quick Reference)

### Initialization
- `begin()`  
  Initializes the sensor, loads calibration data, and applies default configuration.

- `softReset()`  
  Resets the BME280 sensor to its default state using a software reset command.

- `getChipID()`  
  Reads the chip ID register to confirm correct sensor detection (expected value: `0x60`).

---

### Power Modes
- `setMode(mode)`  
  Sets the sensor operating mode (`SLEEP`, `FORCED`, or `NORMAL`) based on Bosch power mode macros.

- `getMode()`  
  Returns the currently active power mode from the sensor.

---

### Sensor Read Functions
- `readTemperature(temp)`  
  Reads compensated temperature in **°C** and stores the result in the provided variable.

- `readPressure(pressure)`  
  Reads compensated pressure in **hPa** and stores the result in the provided variable.

- `readHumidity(humidity)`  
  Reads compensated humidity in **%RH** and stores the result in the provided variable.

- `readLowPower()`  
  Triggers a single forced measurement and updates internal cached readings for low-power applications.

---

### Configuration
- `setOversampling(osrT, osrP, osrH)`  
  Sets oversampling for temperature, pressure, and humidity to balance accuracy vs power usage.

- `setFilter(filter)`  
  Enables or disables the IIR filter to smooth noisy pressure and temperature readings.

- `setStandby(standby)`  
  Sets standby delay between measurements (used only in **NORMAL mode**).

- `getMeasurementDelay()`  
  Returns the estimated measurement time based on current oversampling settings.

---

### Helpers
- `readAltitude(seaLevelhPa)`  
  Calculates altitude in meters using pressure and reference sea-level pressure (default `1013.25 hPa`).

- `getStatus()`  
  Reads the status register to check measurement progress and internal memory updates.

- **I2C (SDA, SCL)** or **SPI (MOSI, MISO, SCK, CS)** connection  

---

## Getting Started

### 1. Library Installation

The recommended way to install this library is via the **Arduino Library Manager**:

1. Open the **Arduino IDE** (v1.8.13+ or Arduino IDE 2.x)  
2. Navigate to  
   - `Sketch` > `Include Library` > `Manage Libraries…` (Arduino IDE 1.x), or  
   - `Tools` > `Manage Libraries…` or click 📚 (IDE 2.x sidebar)
3. Search for:
   " 7Semi BME690 "
4. Click **Install**

Once installed, include it in your sketch:
```cpp
" #include <7semi_bme690.h> "

🔎 Tip: You can find all 7Semi libraries by just searching for 7Semi in the Library Manager.

