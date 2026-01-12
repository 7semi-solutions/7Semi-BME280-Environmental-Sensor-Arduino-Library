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
 * @file    7Semi_BME280.cpp
 * @brief   Arduino-compatible BME280 wrapper implementation
 */

#include "7Semi_BME280.h"
#include <math.h>

/**
 * Constructor : I2C
 *
 * - Stores bus parameters into class variables
 * - Starts I2C bus with optional SDA/SCL pins
 * - Sets I2C clock speed
 *
 * Notes:
 * - Some boards (ESP32 / RP2040) support custom I2C pins.
 * - Other boards ignore SDA/SCL values and use default pins.
 */
BMI280_7Semi::BMI280_7Semi(uint8_t i2c_addr,
                           uint8_t i2c_sda,
                           uint8_t i2c_scl,
                           uint32_t i2c_speed,
                           TwoWire &wire)
{
    bus = I2C;

    address = i2c_addr;
    sda = i2c_sda;
    scl = i2c_scl;
    speed = i2c_speed;

    i2c = &wire;

#if defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
    if (sda != 0xFF && scl != 0xFF)
        i2c->begin(sda, scl);
    else
        i2c->begin();
#else
    i2c->begin();
#endif

    i2c->setClock(speed);
}

/**
 * Constructor : SPI
 *
 * - Stores bus parameters into class variables
 * - Initializes chip-select pin and sets it HIGH (idle state)
 * - Starts SPI bus with optional custom pin mapping
 *
 * Notes:
 * - On ESP32, SPIClass supports begin(SCK, MISO, MOSI, SS).
 * - On other boards, begin() uses default pins.
 */
BMI280_7Semi::BMI280_7Semi(uint8_t spi_cs,
                           uint8_t spi_miso,
                           uint8_t spi_mosi,
                           uint8_t spi_sck,
                           uint32_t spi_speed_hz,
                           SPIClass &spi_bus)
{
    bus = SPI;

    cs_pin = spi_cs;
    miso = spi_miso;
    mosi = spi_mosi;
    sck = spi_sck;
    spi_speed = spi_speed_hz;

    spi = &spi_bus;

    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

#if defined(ESP32)
    if (sck != 0xFF && miso != 0xFF && mosi != 0xFF)
        spi->begin(sck, miso, mosi, cs_pin);
    else
        spi->begin();
#else
    spi->begin();
#endif
}

/**
 * begin()
 *
 * - Clears Bosch structures
 * - Assigns correct interface type and callback functions
 * - Calls bme280_init() to load calibration and verify communication
 * - Applies default oversampling/filter/standby settings
 * - Starts sensor in NORMAL mode by default
 *
 * Default configuration:
 * - Temp  : Oversampling 2X
 * - Press : Oversampling 16X
 * - Hum   : Oversampling 1X
 * - Filter: 16
 * - Standby: 125 ms
 */
bool BMI280_7Semi::begin()
{
    memset(&dev, 0, sizeof(dev));
    memset(&settings, 0, sizeof(settings));
    memset(&data, 0, sizeof(data));

    /**
     * Configure Bosch interface based on selected bus
     */
    if (bus == SPI)
    {
        pinMode(cs_pin, OUTPUT);
        digitalWrite(cs_pin, HIGH);

        dev.intf = BME280_SPI_INTF;
        dev.read = spi_read;
        dev.write = spi_write;
    }
    else
    {
        dev.intf = BME280_I2C_INTF;
        dev.read = i2c_read;
        dev.write = i2c_write;
    }

    /**
     * Assign delay and instance pointer for Bosch driver
     */
    dev.delay_us = delay_us;
    dev.intf_ptr = this;

    /**
     * Initialize Bosch driver and sensor
     */
    lastError = bme280_init(&dev);
    if (lastError != BME280_OK)
        return false;

    /**
     * Apply default sensor settings
     */
    settings.osr_t = BME280_OVERSAMPLING_2X;
    settings.osr_p = BME280_OVERSAMPLING_16X;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.filter = BME280_FILTER_COEFF_16;
    settings.standby_time = BME280_STANDBY_TIME_125_MS;

    lastError = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &dev);
    if (lastError != BME280_OK)
        return false;

    return setMode(BME280_POWERMODE_NORMAL);
}

/**
 * softReset()
 *
 * - Issues a Bosch software reset command
 * - Sensor returns to default state
 * - Recommended to call begin() again after reset
 */
bool BMI280_7Semi::softReset()
{
    lastError = bme280_soft_reset(&dev);
    return lastError == BME280_OK;
}

/**
 * getChipID()
 *
 * - Reads BME280 chip ID register
 * - Expected value is 0x60
 */
uint8_t BMI280_7Semi::getChipID()
{
    uint8_t id = 0;
    bme280_get_regs(BME280_REG_CHIP_ID, &id, 1, &dev);
    return id;
}

/**
 * setMode()
 *
 * - Sets BME280 power mode
 * - Use:
 *   - BME280_POWERMODE_SLEEP
 *   - BME280_POWERMODE_FORCED
 *   - BME280_POWERMODE_NORMAL
 */
bool BMI280_7Semi::setMode(uint8_t mode)
{
    lastError = bme280_set_sensor_mode(mode, &dev);
    return lastError == BME280_OK;
}

/**
 * getMode()
 *
 * - Reads current sensor power mode
 */
uint8_t BMI280_7Semi::getMode()
{
    uint8_t mode = 0;
    lastError = bme280_get_sensor_mode(&mode, &dev);
    return mode;
}

/**
 * getStatus()
 *
 * - Reads BME280 status register
 * - Useful for checking:
 *   - ongoing measurement
 *   - NVM copy status
 */
uint8_t BMI280_7Semi::getStatus()
{
    uint8_t s = 0;
    bme280_get_regs(BME280_REG_STATUS, &s, 1, &dev);
    return s;
}

/**
 * readAll()
 *
 * - Reads temperature, pressure and humidity into internal `data`
 * - If forced mode is active, waits for conversion before reading
 *
 * Notes:
 * - This is the base function for individual read methods.
 */
bool BMI280_7Semi::readAll()
{
    /**
     * If sensor is busy, return false to avoid reading invalid values
     */
    lastError = getStatus();
    if (lastError & 0x01)
        return false;

    /**
     * If forced mode, wait for measurement completion
     */
    if (getMode() == BME280_POWERMODE_FORCED)
        delay(getMeasurementDelay());

    lastError = bme280_get_sensor_data(BME280_ALL, &data, &dev);
    return lastError == BME280_OK;
}

/**
 * readLowPower()
 *
 * - Intended for low power battery use cases
 * - Uses forced mode for single conversion
 * - Waits for conversion delay and then reads all values
 */
bool BMI280_7Semi::readLowPower()
{
    lastError = setMode(BME280_POWERMODE_FORCED);
    if (lastError != BME280_OK)
        return false;

    delay(getMeasurementDelay());
    return readAll();
}

/**
 * readTemperature()
 *
 * - Reads and returns compensated temperature in °C
 * - Bosch driver returns scaled values if DOUBLE mode is disabled
 */
bool BMI280_7Semi::readTemperature(float &temperature)
{
    if (!readAll())
        return false;

#ifdef BME280_DOUBLE_ENABLE
    temperature = data.temperature;
#else
    temperature = data.temperature / 100.0f;
#endif

    return true;
}

/**
 * readPressure()
 *
 * - Reads and returns pressure in hPa
 * - Bosch driver returns pressure in Pa scale internally
 */
bool BMI280_7Semi::readPressure(float &pressure)
{
    if (!readAll())
        return false;

    pressure = data.pressure / 100.0f;
    return true;
}

/**
 * readHumidity()
 *
 * - Reads and returns humidity in %RH
 * - Bosch driver returns scaled values if DOUBLE mode is disabled
 */
bool BMI280_7Semi::readHumidity(float &humidity)
{
    if (!readAll())
        return false;

#ifdef BME280_DOUBLE_ENABLE
    humidity = data.humidity;
#else
    humidity = data.humidity / 1024.0f;
#endif

    return true;
}

/**
 * setOversampling()
 *
 * - Updates oversampling settings for temperature, pressure, humidity
 * - Reads current mode, applies settings, restores previous mode
 *
 * Notes:
 * - Bosch driver internally switches to sleep before modifying settings
 * - Restoring mode ensures measurement behavior remains unchanged
 */
bool BMI280_7Semi::setOversampling(uint8_t osrT, uint8_t osrP, uint8_t osrH)
{
    uint8_t prev_mode = 0;

    lastError = bme280_get_sensor_mode(&prev_mode, &dev);
    if (lastError != BME280_OK)
        return false;

    settings.osr_t = osrT;
    settings.osr_p = osrP;
    settings.osr_h = osrH;

    lastError = bme280_set_sensor_settings(
        BME280_SEL_OSR_TEMP |
            BME280_SEL_OSR_PRESS |
            BME280_SEL_OSR_HUM,
        &settings, &dev);

    if (lastError != BME280_OK)
        return false;

    return setMode(prev_mode);
}

/**
 * setFilter()
 *
 * - Updates IIR filter coefficient
 * - Restores previous mode after applying
 */
bool BMI280_7Semi::setFilter(uint8_t filter)
{
    uint8_t prev_mode = 0;

    lastError = bme280_get_sensor_mode(&prev_mode, &dev);
    if (lastError != BME280_OK)
        return false;

    settings.filter = filter;

    lastError = bme280_set_sensor_settings(BME280_SEL_FILTER, &settings, &dev);
    if (lastError != BME280_OK)
        return false;

    return setMode(prev_mode);
}

/**
 * setStandby()
 *
 * - Updates standby time (only relevant for NORMAL mode)
 * - Restores previous mode after applying
 */
bool BMI280_7Semi::setStandby(uint8_t standby)
{
    uint8_t prev_mode = 0;

    lastError = bme280_get_sensor_mode(&prev_mode, &dev);
    if (lastError != BME280_OK)
        return false;

    settings.standby_time = standby;

    lastError = bme280_set_sensor_settings(BME280_SEL_STANDBY, &settings, &dev);
    if (lastError != BME280_OK)
        return false;

    return setMode(prev_mode);
}

/**
 * getMeasurementDelay()
 *
 * - Computes required conversion delay based on oversampling settings
 * - Used for forced mode wait timing
 *
 * Notes:
 * - The Bosch API returns delay in microseconds
 * - Arduino delay() requires milliseconds
 */
uint32_t BMI280_7Semi::getMeasurementDelay()
{
    uint32_t delay_us_val = 0;
    bme280_cal_meas_delay(&delay_us_val, &settings);

    /**
     * Convert microseconds → milliseconds
     * - Ensure minimum 1ms delay
     */
    uint32_t delay_ms_val = delay_us_val / 1000;
    if (delay_ms_val == 0)
        delay_ms_val = 1;

    return delay_ms_val;
}

/**
 * readAltitude()
 *
 * - Uses pressure reading to estimate altitude
 * - Formula based on international barometric formula
 * - Returns altitude in meters
 */
float BMI280_7Semi::readAltitude(float seaLevelhPa)
{
    float p = 0;
    if (!readPressure(p))
        return NAN;

    return 44330.0f * (1.0f - pow(p / seaLevelhPa, 0.1903f));
}

/**
 * I2C Write Callback
 *
 * - Bosch driver calls this when it needs to write registers over I2C
 */
int8_t BMI280_7Semi::i2c_write(uint8_t reg,
                               const uint8_t *data,
                               uint32_t len,
                               void *ptr)
{
    auto *bmi280 = (BMI280_7Semi *)ptr;

    bmi280->i2c->beginTransmission(bmi280->address);
    bmi280->i2c->write(reg);

    for (uint32_t i = 0; i < len; i++)
        bmi280->i2c->write(data[i]);

    uint8_t result = bmi280->i2c->endTransmission();
    if (result != 0)
        return -1;

    return BME280_INTF_RET_SUCCESS;
}

/**
 * I2C Read Callback
 *
 * - Bosch driver calls this when it needs to read registers over I2C
 */
int8_t BMI280_7Semi::i2c_read(uint8_t reg,
                              uint8_t *data,
                              uint32_t len,
                              void *ptr)
{
    auto *bmi280 = (BMI280_7Semi *)ptr;

    bmi280->i2c->beginTransmission(bmi280->address);
    bmi280->i2c->write(reg);

    if (bmi280->i2c->endTransmission(false) != 0)
        return -1;

    uint32_t received = bmi280->i2c->requestFrom(bmi280->address, (uint8_t)len);
    if (received != len)
        return -1;

    for (uint32_t i = 0; i < len; i++)
        data[i] = bmi280->i2c->read();

    return BME280_INTF_RET_SUCCESS;
}

/**
 * SPI Read Callback
 *
 * - Bosch driver calls this when it needs to read registers over SPI
 * - The MSB bit must be set to indicate read
 */
int8_t BMI280_7Semi::spi_read(uint8_t reg,
                              uint8_t *data,
                              uint32_t len,
                              void *ptr)
{
    if (!ptr || !data || len == 0)
        return -1;

    auto *bmi280 = (BMI280_7Semi *)ptr;

    digitalWrite(bmi280->cs_pin, LOW);

    bmi280->spi->transfer(reg | 0x80);

    for (uint32_t i = 0; i < len; i++)
        data[i] = bmi280->spi->transfer(0x00);

    digitalWrite(bmi280->cs_pin, HIGH);

    return BME280_INTF_RET_SUCCESS;
}

/**
 * SPI Write Callback
 *
 * - Bosch driver calls this when it needs to write registers over SPI
 * - The MSB bit must be cleared to indicate write
 */
int8_t BMI280_7Semi::spi_write(uint8_t reg,
                               const uint8_t *data,
                               uint32_t len,
                               void *ptr)
{
    auto *bmi280 = (BMI280_7Semi *)ptr;

    digitalWrite(bmi280->cs_pin, LOW);

    bmi280->spi->transfer(reg & 0x7F);

    for (uint32_t i = 0; i < len; i++)
        bmi280->spi->transfer(data[i]);

    digitalWrite(bmi280->cs_pin, HIGH);

    return BME280_INTF_RET_SUCCESS;
}

/**
 * delay_us()
 *
 * - Bosch driver expects delays in microseconds
 * - Arduino provides delayMicroseconds()
 */
void BMI280_7Semi::delay_us(uint32_t us, void *)
{
    delayMicroseconds(us);
}
