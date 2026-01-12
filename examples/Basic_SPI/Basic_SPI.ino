/**
 * 7Semi BME280 - SPI Basic Example
 *
 * - Reads temperature, pressure, and humidity using SPI interface
 *
 * Wiring (SPI):
 * - CS   -> any GPIO (example uses GPIO 10)
 * - SCK  -> SCK
 * - MOSI -> MOSI
 * - MISO -> MISO
 * - VCC  -> 3.3V / 5V (depends on module)
 * - GND  -> GND
 */

#include <7Semi_BME280.h>

uint8_t spi_cs = 5;    // Chip Select pin
uint8_t spi_miso = -1;  // Arduino MISO pin D12
uint8_t spi_mosi = -1;  // Arduino MOSI pin D11
uint8_t spi_sck = -1;   // Arduino SCK pin D13
uint32_t spi_speed = 1000000;

// for ESP32
// uint8_t spi_cs = 5;          // Chip Select pin
// uint8_t spi_miso = -1;         // Arduino MISO pin D19
// uint8_t spi_mosi = -1;         // Arduino MOSI pin D23
// uint8_t spi_sck = -1;         // Arduino SCK pin D18
// uint32_t spi_speed = 1000000;


/* Create sensor object using SPI with CS pin */
BMI280_7Semi bme(spi_cs, spi_miso, spi_mosi, spi_sck, spi_speed, SPI);

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("7Semi BME280 SPI Example");

  /* Initialize sensor */
  if (!bme.begin()) {
    Serial.println("BME280 init failed!");
    while (1)
      delay(100);
  }

  Serial.print("Chip ID: 0x");
  Serial.println(bme.getChipID(), HEX);
}

void loop() {
  float t, p, h;

  /* Read temperature */
  if (bme.readTemperature(t)) {
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" °C");
  } else {
    Serial.println("Temperature read failed!");
  }

  /* Read pressure */
  if (bme.readPressure(p)) {
    Serial.print("Pressure: ");
    Serial.print(p);
    Serial.println(" hPa");
  } else {
    Serial.println("Pressure read failed!");
  }

  /* Read humidity */
  if (bme.readHumidity(h)) {
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.println(" %RH");
  } else {
    Serial.println("Humidity read failed!");
  }

  Serial.println("---------------------");

  delay(1000);
}
