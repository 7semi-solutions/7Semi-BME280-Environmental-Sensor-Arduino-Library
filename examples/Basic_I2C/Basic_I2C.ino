/**
 * 7Semi BME280 - I2C Basic Example
 *
 * - Reads temperature, pressure, and humidity from BME280
 *
 * Wiring (default I2C):
 * - SDA -> SDA pin of board
 * - SCL -> SCL pin of board
 * - VCC -> 3.3V / 5V (depends on module)
 * - GND -> GND
 */

#include <7Semi_BME280.h>

uint8_t i2c_address = 0x77;   // BME280 I2C address (0x76 or 0x77)
uint8_t i2c_sda = -1;         // Not used in I2C mode
uint8_t i2c_scl = -1;         // Not used in I2C mode
uint32_t i2c_speed = 100000;  // Not used in I2C mode
/* Create sensor object (default address 0x76) */
BMI280_7Semi bme(i2c_address, i2c_sda, i2c_scl, i2c_speed, Wire);

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("7Semi BME280 I2C Example");

  /* Initialize sensor */
  if (!bme.begin()) {
    Serial.println("BME280 init failed!");
    while (1)
      delay(100);
  }
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


