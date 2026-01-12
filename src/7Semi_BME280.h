/**
 * Copyright (c) 2026 7Semi.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * --------------------------------------------------------------------------
 *
 * This library is an Arduino-compatible wrapper for the Bosch Sensortec BME280
 * Sensor API.
 *
 * Credits:
 * - Bosch Sensortec GmbH BME280 Sensor API (BSD-3-Clause Licensed)
 * - Source: https://github.com/BoschSensortec/BME280_driver
 *
 * Notes:
 * - This project is not affiliated with or endorsed by Bosch Sensortec GmbH.
 * - The Bosch driver license text is preserved in the Bosch driver files.
 *
 * --------------------------------------------------------------------------
 *
 * @file    7semi_bme280.h
 * @date    2026-01-06
 * @brief   Arduino-compatible BME280 wrapper (I2C + SPI)
 */

#ifndef _7SEMI_BME280_H_
#define _7SEMI_BME280_H_

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

extern "C"
{
#include "bme280.h"
}

/**
 * 7Semi BME280 Sensor Wrapper
 *
 * - Supports both I2C and SPI communication
 * - Uses Bosch Sensortec official BME280 driver internally
 * - Provides easy Arduino-friendly methods for sensor setup and readout
 *
 * Notes:
 * - The naming is kept as BMI280_7Semi based on your implementation
 * - Internally uses Bosch's `bme280_dev` and callback-based interface
 */
class BMI280_7Semi
{
public:
    /**
     * Create an I2C instance of the sensor driver
     *
     * - Default I2C address: 0x76
     * - SDA/SCL pins can be provided for boards supporting Wire.begin(sda,scl)
     * - Default speed is 100 kHz
     * - Uses the supplied TwoWire object
     *
     * Typical use:
     * - BMI280_7Semi bme;
     * - bme.begin();
     */
    BMI280_7Semi(uint8_t i2c_addr = 0x76,
                 uint8_t i2c_sda = 0xFF,
                 uint8_t i2c_scl = 0xFF,
                 uint32_t i2c_speed = 100000,
                 TwoWire &wire = Wire);

    /**
     * Create an SPI instance of the sensor driver
     *
     * - Default CS pin: 10
     * - MISO/MOSI/SCK pins can be provided for boards supporting custom SPI pins
     * - Default speed is 1 MHz
     * - Uses the supplied SPIClass object
     *
     * Typical use:
     * - BMI280_7Semi bme(10);
     * - bme.begin();
     */
    BMI280_7Semi(uint8_t spi_cs = 10,
                 uint8_t spi_miso = 0xFF,
                 uint8_t spi_mosi = 0xFF,
                 uint8_t spi_sck = 0xFF,
                 uint32_t spi_speed = 1000000,
                 SPIClass &spi_bus = ::SPI);

    /**
     * Initialize the sensor and configure the Bosch driver
     *
     * - Detects chip, checks communication, and loads calibration values
     * - Must be called before any read methods
     * - Returns true only if sensor initialization is successful
     */
    bool begin();

    /**
     * Perform a software reset of the BME280 sensor
     *
     * - Restores sensor to default state
     * - Calibration must be reloaded by re-initializing or reading again
     */
    bool softReset();

    /**
     * Read and return the sensor chip ID
     *
     * - Useful to verify correct sensor connection
     * - Expected chip ID for BME280 is 0x60
     */
    uint8_t getChipID();

    /**
     * Set BME280 power mode
     *
     * - Use Bosch-defined modes:
     *   - BME280_SLEEP_MODE
     *   - BME280_FORCED_MODE
     *   - BME280_NORMAL_MODE
     *
     * Notes:
     * - Forced mode triggers a single measurement
     * - Normal mode continuously measures based on standby time
     */
    bool setMode(uint8_t mode);

    /**
     * Read current sensor power mode
     *
     * - Returns the mode currently active in the BME280 sensor
     */
    uint8_t getMode();

    /**
     * Read temperature value
     *
     * - Output in °C
     * - Returns true if successful
     */
    bool readTemperature(float &temperature);

    /**
     * Read pressure value
     *
     * - Output in hPa (hectopascal)
     * - Returns true if successful
     */
    bool readPressure(float &pressure);

    /**
     * Read humidity value
     *
     * - Output in % RH
     * - Returns true if successful
     */
    bool readHumidity(float &humidity);

    /**
     * Low power measurement read
     *
     * - Designed for battery-powered or sleep-heavy applications
     * - Internally sets forced mode + reads all values
     */
    bool readLowPower();

    /**
     * Set sensor oversampling configuration
     *
     * - Controls resolution, power consumption, and conversion time
     * - Suggested:
     *   - Temperature: BME280_OVERSAMPLING_1X
     *   - Pressure:    BME280_OVERSAMPLING_1X
     *   - Humidity:    BME280_OVERSAMPLING_1X
     */
    bool setOversampling(uint8_t osrT, uint8_t osrP, uint8_t osrH);

    /**
     * Enable and configure IIR filter
     *
     * - Reduces noise in pressure and temperature readings
     * - Use values:
     *   - BME280_FILTER_COEFF_OFF
     *   - BME280_FILTER_COEFF_2
     *   - BME280_FILTER_COEFF_4
     *   - BME280_FILTER_COEFF_8
     *   - BME280_FILTER_COEFF_16
     */
    bool setFilter(uint8_t filter);

    /**
     * Set standby duration (Normal mode)
     *
     * - Defines the wait time between measurements in normal mode
     * - Use Bosch-defined values:
     *   - BME280_STANDBY_TIME_0_5_MS
     *   - BME280_STANDBY_TIME_62_5_MS
     *   - BME280_STANDBY_TIME_125_MS
     *   - BME280_STANDBY_TIME_250_MS
     *   - BME280_STANDBY_TIME_500_MS
     *   - BME280_STANDBY_TIME_1000_MS
     *   - BME280_STANDBY_TIME_10_MS
     *   - BME280_STANDBY_TIME_20_MS
     */
    bool setStandby(uint8_t standby);

    /**
     * Return measurement delay time in microseconds
     *
     * - Based on oversampling settings
     * - Useful for forced mode conversion wait
     */
    uint32_t getMeasurementDelay();

    /**
     * Calculate altitude based on pressure and sea-level reference
     *
     * - Default sea level pressure: 1013.25 hPa
     * - Returns altitude in meters
     */
    float readAltitude(float seaLevelhPa = 1013.25);

    /**
     * Read the BME280 status register
     *
     * - Allows checking if measurement is running or NVM copy is busy
     */
    uint8_t getStatus();

private:
    /**
     * Bosch driver structures
     *
     * - `dev` stores calibration + callbacks
     * - `settings` stores oversampling/filter/standby configuration
     * - `data` stores latest sensor readings from Bosch driver
     */
    struct bme280_dev dev;
    struct bme280_settings settings;
    struct bme280_data data;

    /**
     * Cached last error status
     *
     * - Bosch driver returns BME280_OK on success
     * - Any other value indicates a communication or configuration error
     */
    int8_t lastError = BME280_OK;

    /**
     * I2C configuration
     *
     * - address: sensor I2C address (0x76 or 0x77)
     * - sda/scl: custom pins (optional)
     * - speed: bus speed (Hz)
     */
    uint8_t address;
    uint8_t sda;
    uint8_t scl;
    uint32_t speed;
    TwoWire *i2c;

    /**
     * SPI configuration
     *
     * - cs_pin: chip select pin
     * - miso/mosi/sck: SPI pins (optional)
     * - spi_speed: SPI clock speed (Hz)
     */
    uint8_t cs_pin;
    uint8_t miso;
    uint8_t mosi;
    uint8_t sck;
    uint32_t spi_speed;
    SPIClass *spi;

    /**
     * Active bus type selection
     *
     * - I2C: Uses Wire-like interface
     * - SPI: Uses SPIClass interface
     */
    enum BusType
    {
        I2C,
        SPI
    } bus;

    /**
     * I2C read callback for Bosch driver
     *
     * - reg: start register
     * - data: buffer to store received bytes
     * - len: number of bytes to read
     */
    static int8_t i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *ptr);

    /**
     * I2C write callback for Bosch driver
     *
     * - reg: start register
     * - data: buffer containing bytes to transmit
     * - len: number of bytes to write
     */
    static int8_t i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *ptr);

    /**
     * SPI read callback for Bosch driver
     *
     * - reg: start register
     * - data: buffer to store received bytes
     * - len: number of bytes to read
     */
    static int8_t spi_read(uint8_t reg, uint8_t *data, uint32_t len, void *ptr);

    /**
     * SPI write callback for Bosch driver
     *
     * - reg: start register
     * - data: buffer containing bytes to transmit
     * - len: number of bytes to write
     */
    static int8_t spi_write(uint8_t reg, const uint8_t *data, uint32_t len, void *ptr);

    /**
     * Delay callback for Bosch driver
     *
     * - Bosch driver expects delay in microseconds
     * - Most Arduino implementations use delayMicroseconds()
     */
    static void delay_us(uint32_t period, void *);

    /**
     * Read all sensor data (temperature, pressure, humidity)
     *
     * - Updates internal `data` struct
     * - Used as the base for individual read calls
     */
    bool readAll();
};

#endif
